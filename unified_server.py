import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'web'))

from flask import Blueprint, send_from_directory, jsonify, Response

from app import app

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DOCS_DIR = os.path.join(BASE_DIR, 'docs')
RV32_DIR = os.path.join(BASE_DIR, 'riscv_cap')

rv32_bp = Blueprint('rv32', __name__, url_prefix='/rv32')

@rv32_bp.route('/')
def rv32_index():
    return send_from_directory(RV32_DIR, 'index.html')

@rv32_bp.route('/api/docs')
def rv32_list_docs():
    files = []
    if os.path.isdir(DOCS_DIR):
        for f in sorted(os.listdir(DOCS_DIR)):
            if f.endswith('.md'):
                path = os.path.join(DOCS_DIR, f)
                files.append({
                    'name': f,
                    'size': os.path.getsize(path),
                    'title': f.replace('.md', '').replace('-', ' ').title()
                })
    return jsonify(files)

@rv32_bp.route('/api/docs/<path:filename>')
def rv32_get_doc(filename):
    if not filename.endswith('.md'):
        filename += '.md'
    safe_name = os.path.basename(filename)
    filepath = os.path.join(DOCS_DIR, safe_name)
    if os.path.isfile(filepath):
        with open(filepath, 'r') as f:
            content = f.read()
        return Response(content, mimetype='text/plain')
    return Response('Not found', status=404)

@rv32_bp.route('/<path:path>')
def rv32_static(path):
    return send_from_directory(RV32_DIR, path)

app.register_blueprint(rv32_bp)

@app.route('/')
def landing_page():
    return send_from_directory(BASE_DIR, 'landing.html')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
