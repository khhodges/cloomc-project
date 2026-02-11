from flask import Flask, send_from_directory, send_file, jsonify, Response
import os

app = Flask(__name__, static_folder='.', static_url_path='')

DOCS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'docs')

@app.route('/')
def index():
    return send_file('index.html')

@app.route('/api/docs')
def list_docs():
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

@app.route('/api/docs/<path:filename>')
def get_doc(filename):
    if not filename.endswith('.md'):
        filename += '.md'
    safe_name = os.path.basename(filename)
    filepath = os.path.join(DOCS_DIR, safe_name)
    if os.path.isfile(filepath):
        with open(filepath, 'r') as f:
            content = f.read()
        return Response(content, mimetype='text/plain')
    return Response('Not found', status=404)

@app.route('/<path:path>')
def static_files(path):
    return send_from_directory('.', path)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
