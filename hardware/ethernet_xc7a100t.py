from amaranth import *
from amaranth.lib.memory import Memory as LibMemory

from .hw_types import FaultType


# ─── MMIO Register Map ───────────────────────────────────────────────────────
# Base address: 0x40001000  (word-addressed; reg_sel = addr[2:6])
#
#   0x40001000  [ 0] ETH_CTRL    — bit[0] = reset, bit[1] = enable, bit[2] = loopback
#   0x40001004  [ 1] ETH_STATUS  — 0 = link down, 1 = link up, 2 = negotiating
#   0x40001008  [ 2] ETH_TX_LEN  — byte length of frame to transmit (write triggers TX)
#   0x4000100C  [ 3] ETH_RX_LEN  — byte length of last received frame (read-only)
#   0x40001010  [ 4] ETH_IP_ADDR — packed IPv4 address (network byte order)
#   0x40001014  [ 5] ETH_PORT    — UDP port number
#   0x40001018  [ 6] ETH_TX_DATA — frame TX FIFO (write bytes sequentially before ETH_TX_LEN)
#   0x4000101C  [ 7] ETH_RX_DATA — frame RX FIFO (read bytes sequentially)
#
ETH_MMIO_BASE = 0x40001000

ETH_REG_CTRL     = 0
ETH_REG_STATUS   = 1
ETH_REG_TX_LEN   = 2
ETH_REG_RX_LEN   = 3
ETH_REG_IP_ADDR  = 4
ETH_REG_PORT     = 5
ETH_REG_TX_DATA  = 6
ETH_REG_RX_DATA  = 7

ETH_STATUS_DOWN        = 0x0
ETH_STATUS_UP          = 0x1
ETH_STATUS_NEGOTIATING = 0x2

ETH_DEVICE_GT = 0x07800400

ETH_MAX_FRAME = 1518


class EthernetRMIIDriver(Elaboratable):
    """RMII PHY driver for the XC7A100T Ethernet interface.

    100BASE-TX, half-duplex, no TCP/IP stack.

    Ports
    -----
    RMII PHY signals (connect directly to board pins):
      rmii_ref_clk  — 50 MHz ref clock input from PHY
      rmii_crs_dv   — carrier sense / data valid from PHY
      rmii_rxd[1:0] — receive data dibit from PHY
      rmii_txen     — transmit enable to PHY
      rmii_txd[1:0] — transmit data dibit to PHY
      rmii_nrst     — PHY reset (active-LOW)

    TX interface (from EthernetDeviceGT):
      tx_byte       — byte to transmit
      tx_valid      — asserted for one cycle when tx_byte is valid
      tx_ack        — asserted one cycle to acknowledge tx_byte consumed
      tx_done       — asserted one cycle after last byte of frame sent

    RX interface (to EthernetDeviceGT):
      rx_byte       — received byte
      rx_valid      — asserted one cycle when rx_byte is valid
      rx_last       — asserted with rx_valid on last byte of frame

    Status:
      link_up       — 1 when the PHY has carrier

    QMTECH Wukong board PHY pin assignments (XDC constraints file):
        rmii_ref_clk  → W12   (50 MHz ref clock from PHY)
        rmii_crs_dv   → AA11  (carrier sense / data valid)
        rmii_rxd[0]   → Y11   (receive data bit 0)
        rmii_rxd[1]   → Y12   (receive data bit 1)
        rmii_txen     → V11   (transmit enable)
        rmii_txd[0]   → V12   (transmit data bit 0)
        rmii_txd[1]   → W11   (transmit data bit 1)
        rmii_nrst     → U10   (PHY reset, active-LOW)
        mdio_mdc      → U11   (MDIO management clock)
        mdio_mdio     → V10   (MDIO management data)
    """

    def __init__(self, clk_freq=100_000_000):
        self._clk_freq = clk_freq

        self.rmii_ref_clk = Signal()
        self.rmii_crs_dv  = Signal()
        self.rmii_rxd     = Signal(2)
        self.rmii_txen    = Signal()
        self.rmii_txd     = Signal(2)
        self.rmii_nrst    = Signal(init=1)

        self.tx_byte  = Signal(8)
        self.tx_valid = Signal()
        self.tx_ack   = Signal()
        self.tx_done  = Signal()

        self.rx_byte  = Signal(8)
        self.rx_valid = Signal()
        self.rx_last  = Signal()

        self.link_up  = Signal()

    def elaborate(self, platform):
        m = Module()

        phy_reset_cnt = Signal(range(self._clk_freq // 1000 + 1))
        with m.If(phy_reset_cnt < (self._clk_freq // 1000)):
            m.d.sync += phy_reset_cnt.eq(phy_reset_cnt + 1)
            m.d.comb += self.rmii_nrst.eq(0)
        with m.Else():
            m.d.comb += self.rmii_nrst.eq(1)

        m.d.sync += self.link_up.eq(self.rmii_crs_dv)

        rx_dibit_cnt = Signal(2)
        rx_byte_acc  = Signal(8)
        rx_prev_crs  = Signal()

        m.d.sync += rx_prev_crs.eq(self.rmii_crs_dv)

        with m.If(self.rmii_crs_dv):
            with m.Switch(rx_dibit_cnt):
                with m.Case(0): m.d.sync += rx_byte_acc[0:2].eq(self.rmii_rxd)
                with m.Case(1): m.d.sync += rx_byte_acc[2:4].eq(self.rmii_rxd)
                with m.Case(2): m.d.sync += rx_byte_acc[4:6].eq(self.rmii_rxd)
                with m.Case(3):
                    m.d.comb += [
                        self.rx_byte.eq(Cat(rx_byte_acc[:6], self.rmii_rxd)),
                        self.rx_valid.eq(1),
                    ]
            m.d.sync += rx_dibit_cnt.eq(rx_dibit_cnt + 1)
        with m.Else():
            with m.If(rx_prev_crs):
                m.d.comb += self.rx_last.eq(1)
            m.d.sync += rx_dibit_cnt.eq(0)

        tx_active  = Signal()
        tx_byte_r  = Signal(8)
        tx_dibit   = Signal(2)

        with m.If(self.tx_valid & ~tx_active):
            m.d.sync += [
                tx_byte_r.eq(self.tx_byte),
                tx_active.eq(1),
                tx_dibit.eq(0),
            ]
            m.d.comb += self.tx_ack.eq(1)

        with m.If(tx_active):
            m.d.comb += self.rmii_txen.eq(1)
            with m.Switch(tx_dibit):
                with m.Case(0): m.d.comb += self.rmii_txd.eq(tx_byte_r[0:2])
                with m.Case(1): m.d.comb += self.rmii_txd.eq(tx_byte_r[2:4])
                with m.Case(2): m.d.comb += self.rmii_txd.eq(tx_byte_r[4:6])
                with m.Case(3): m.d.comb += self.rmii_txd.eq(tx_byte_r[6:8])
            m.d.sync += tx_dibit.eq(tx_dibit + 1)
            with m.If(tx_dibit == 3):
                m.d.sync += tx_active.eq(0)
                m.d.comb += self.tx_done.eq(1)

        return m


class EthernetDeviceGT(Elaboratable):
    """Ethernet device GT for the XC7A100T — wraps EthernetRMIIDriver.

    Implements the MMIO register interface described in the register map above.

    TX frame path:
      1. Caller writes bytes to ETH_TX_DATA (reg 6) sequentially.
      2. Caller writes frame length to ETH_TX_LEN (reg 2) — this triggers TX.
      3. ETH_STATUS bit[2] (tx_busy) asserts until the frame is fully sent.

    RX frame path:
      1. RMII driver accumulates bytes via rmii.rx_byte + rmii.rx_valid.
      2. When rmii.rx_last fires, ETH_STATUS bit[1] (rx_ready) asserts and
         ETH_RX_LEN (reg 3) holds the complete frame byte count.
      3. Caller reads bytes sequentially from ETH_RX_DATA (reg 7).
      4. After the last RX byte is read, rx_ready clears automatically.

    The frame buffers are Artix-7 BRAM instances (via LibMemory, shape=unsigned(8),
    depth=ETH_MAX_FRAME=1518) — one for TX, one for RX.

    This module is instantiated by ChurchWukongXC7A100T as submodule 'eth' and
    wired into the MMIO decode at base address 0x40001000 (addr[12]=1).
    """

    def __init__(self, clk_freq=100_000_000):
        self._clk_freq = clk_freq

        self.mmio_addr    = Signal(4)
        self.mmio_rd_en   = Signal()
        self.mmio_wr_en   = Signal()
        self.mmio_wr_data = Signal(32)
        self.mmio_rd_data = Signal(32)

        self.rmii_ref_clk = Signal()
        self.rmii_crs_dv  = Signal()
        self.rmii_rxd     = Signal(2)
        self.rmii_txen    = Signal()
        self.rmii_txd     = Signal(2)
        self.rmii_nrst    = Signal(init=1)

        self.eth_irq      = Signal()

    def elaborate(self, platform):
        m = Module()

        rmii = EthernetRMIIDriver(self._clk_freq)
        m.submodules.rmii = rmii

        m.d.comb += [
            rmii.rmii_ref_clk.eq(self.rmii_ref_clk),
            rmii.rmii_crs_dv.eq(self.rmii_crs_dv),
            rmii.rmii_rxd.eq(self.rmii_rxd),
            self.rmii_txen.eq(rmii.rmii_txen),
            self.rmii_txd.eq(rmii.rmii_txd),
            self.rmii_nrst.eq(rmii.rmii_nrst),
        ]

        ctrl_reg    = Signal(3)
        ip_addr_reg = Signal(32)
        port_reg    = Signal(16)

        tx_buf = LibMemory(shape=unsigned(8), depth=ETH_MAX_FRAME, init=[])
        m.submodules.tx_buf = tx_buf
        tx_wr_port = tx_buf.write_port()
        tx_rd_port = tx_buf.read_port(domain="comb")

        tx_wr_ptr    = Signal(range(ETH_MAX_FRAME + 1))
        tx_rd_ptr    = Signal(range(ETH_MAX_FRAME + 1))
        tx_len_reg   = Signal(range(ETH_MAX_FRAME + 1))
        tx_busy      = Signal()
        tx_bytes_out = Signal(range(ETH_MAX_FRAME + 1))

        rx_buf = LibMemory(shape=unsigned(8), depth=ETH_MAX_FRAME, init=[])
        m.submodules.rx_buf = rx_buf
        rx_wr_port = rx_buf.write_port()
        rx_rd_port = rx_buf.read_port(domain="comb")

        rx_wr_ptr  = Signal(range(ETH_MAX_FRAME + 1))
        rx_rd_ptr  = Signal(range(ETH_MAX_FRAME + 1))
        rx_len_reg = Signal(range(ETH_MAX_FRAME + 1))
        rx_ready   = Signal()

        m.d.comb += [
            tx_wr_port.en.eq(0),
            tx_wr_port.addr.eq(tx_wr_ptr),
            tx_wr_port.data.eq(self.mmio_wr_data[:8]),
            tx_rd_port.addr.eq(tx_rd_ptr),

            rx_wr_port.en.eq(0),
            rx_wr_port.addr.eq(rx_wr_ptr),
            rx_wr_port.data.eq(rmii.rx_byte),
            rx_rd_port.addr.eq(rx_rd_ptr),
        ]

        with m.If(self.mmio_wr_en):
            with m.Switch(self.mmio_addr):
                with m.Case(ETH_REG_CTRL):
                    m.d.sync += ctrl_reg.eq(self.mmio_wr_data[:3])

                with m.Case(ETH_REG_TX_DATA):
                    with m.If(~tx_busy & (tx_wr_ptr < ETH_MAX_FRAME)):
                        m.d.comb += tx_wr_port.en.eq(1)
                        m.d.sync += tx_wr_ptr.eq(tx_wr_ptr + 1)

                with m.Case(ETH_REG_TX_LEN):
                    m.d.sync += [
                        tx_len_reg.eq(self.mmio_wr_data[:11]),
                        tx_busy.eq(1),
                        tx_rd_ptr.eq(0),
                        tx_bytes_out.eq(0),
                    ]

                with m.Case(ETH_REG_IP_ADDR):
                    m.d.sync += ip_addr_reg.eq(self.mmio_wr_data)

                with m.Case(ETH_REG_PORT):
                    m.d.sync += port_reg.eq(self.mmio_wr_data[:16])

        m.d.comb += [
            rmii.tx_byte.eq(tx_rd_port.data),
            rmii.tx_valid.eq(tx_busy & (tx_bytes_out < tx_len_reg)),
        ]

        with m.If(rmii.tx_ack):
            m.d.sync += [
                tx_rd_ptr.eq(tx_rd_ptr + 1),
                tx_bytes_out.eq(tx_bytes_out + 1),
            ]

        with m.If(rmii.tx_done & (tx_bytes_out >= tx_len_reg)):
            m.d.sync += [
                tx_busy.eq(0),
                tx_wr_ptr.eq(0),
            ]

        with m.If(rmii.rx_valid & ~rx_ready):
            with m.If(rx_wr_ptr < ETH_MAX_FRAME):
                m.d.comb += rx_wr_port.en.eq(1)
                m.d.sync += rx_wr_ptr.eq(rx_wr_ptr + 1)

        with m.If(rmii.rx_last & (rx_wr_ptr > 0)):
            m.d.sync += [
                rx_len_reg.eq(rx_wr_ptr),
                rx_ready.eq(1),
                rx_rd_ptr.eq(0),
            ]

        last_rx_read = Signal()
        m.d.comb += last_rx_read.eq(
            self.mmio_rd_en &
            (self.mmio_addr == ETH_REG_RX_DATA) &
            ((rx_rd_ptr + 1) >= rx_len_reg)
        )

        with m.If(self.mmio_rd_en & (self.mmio_addr == ETH_REG_RX_DATA)):
            with m.If(rx_rd_ptr < rx_len_reg):
                m.d.sync += rx_rd_ptr.eq(rx_rd_ptr + 1)

        with m.If(last_rx_read):
            m.d.sync += [
                rx_ready.eq(0),
                rx_wr_ptr.eq(0),
            ]

        # ETH_STATUS: direct 3-state value (not a bit-field)
        #   0 = link down  (cable disconnected or no signal)
        #   1 = link up    (ready to send and receive)
        #   2 = negotiating (ctrl_reg[1]=enable set, PHY not yet locked)
        negotiating = Signal()
        m.d.comb += negotiating.eq(ctrl_reg[1] & ~rmii.link_up)

        status_word = Signal(32)
        with m.If(rmii.link_up):
            m.d.comb += status_word.eq(ETH_STATUS_UP)
        with m.Elif(negotiating):
            m.d.comb += status_word.eq(ETH_STATUS_NEGOTIATING)
        with m.Else():
            m.d.comb += status_word.eq(ETH_STATUS_DOWN)

        with m.If(self.mmio_rd_en):
            with m.Switch(self.mmio_addr):
                with m.Case(ETH_REG_CTRL):
                    m.d.comb += self.mmio_rd_data.eq(ctrl_reg)
                with m.Case(ETH_REG_STATUS):
                    m.d.comb += self.mmio_rd_data.eq(status_word)
                with m.Case(ETH_REG_TX_LEN):
                    m.d.comb += self.mmio_rd_data.eq(tx_len_reg)
                with m.Case(ETH_REG_RX_LEN):
                    m.d.comb += self.mmio_rd_data.eq(rx_len_reg)
                with m.Case(ETH_REG_IP_ADDR):
                    m.d.comb += self.mmio_rd_data.eq(ip_addr_reg)
                with m.Case(ETH_REG_PORT):
                    m.d.comb += self.mmio_rd_data.eq(port_reg)
                with m.Case(ETH_REG_RX_DATA):
                    m.d.comb += self.mmio_rd_data.eq(rx_rd_port.data)
                with m.Default():
                    m.d.comb += self.mmio_rd_data.eq(0)

        m.d.comb += self.eth_irq.eq(rx_ready)

        return m
