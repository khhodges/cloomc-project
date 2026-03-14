import signal

bind = "0.0.0.0:5000"
workers = 1
threads = 8
timeout = 120
max_requests = 1000
max_requests_jitter = 50
loglevel = "info"
accesslog = "-"
errorlog = "-"

def when_ready(server):
    signal.signal(signal.SIGWINCH, signal.SIG_IGN)

def post_fork(server, worker):
    signal.signal(signal.SIGWINCH, signal.SIG_IGN)
