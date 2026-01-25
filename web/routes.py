from flask import session, jsonify, request, send_from_directory
from app import app, db
from replit_auth import require_login, make_replit_blueprint
from flask_login import current_user
from models import SimulatorState, LandingPageContent
import json
import os
import logging
import bleach

logger = logging.getLogger(__name__)

try:
    replit_bp = make_replit_blueprint()
    app.register_blueprint(replit_bp, url_prefix="/auth")
    auth_enabled = True
    logger.info("Replit Auth enabled")
except Exception as e:
    auth_enabled = False
    logger.warning(f"Replit Auth not available: {e}")

@app.before_request
def make_session_permanent():
    session.permanent = True

@app.route('/')
def index():
    return send_from_directory('.', 'index.html')

@app.route('/api/user')
def get_user():
    if current_user.is_authenticated:
        return jsonify({
            'authenticated': True,
            'id': current_user.id,
            'email': current_user.email,
            'first_name': current_user.first_name,
            'last_name': current_user.last_name,
            'profile_image_url': current_user.profile_image_url
        })
    return jsonify({'authenticated': False})

@app.route('/api/state', methods=['GET'])
@require_login
def get_state():
    name = request.args.get('name', 'default')
    state = SimulatorState.query.filter_by(user_id=current_user.id, name=name).first()
    if state:
        return jsonify({
            'found': True,
            'name': state.name,
            'state_data': state.state_data,
            'assembly_code': state.assembly_code,
            'updated_at': state.updated_at.isoformat() if state.updated_at else None
        })
    return jsonify({'found': False})

@app.route('/api/state', methods=['POST'])
@require_login
def save_state():
    data = request.get_json()
    name = data.get('name', 'default')
    state_data = data.get('state_data', '{}')
    assembly_code = data.get('assembly_code', '')
    
    state = SimulatorState.query.filter_by(user_id=current_user.id, name=name).first()
    if state:
        state.state_data = state_data
        state.assembly_code = assembly_code
    else:
        state = SimulatorState(
            user_id=current_user.id,
            name=name,
            state_data=state_data,
            assembly_code=assembly_code
        )
        db.session.add(state)
    
    db.session.commit()
    return jsonify({'success': True, 'id': state.id})

@app.route('/api/states', methods=['GET'])
@require_login
def list_states():
    states = SimulatorState.query.filter_by(user_id=current_user.id).all()
    return jsonify({
        'states': [{
            'id': s.id,
            'name': s.name,
            'updated_at': s.updated_at.isoformat() if s.updated_at else None
        } for s in states]
    })

@app.route('/api/state/<int:state_id>', methods=['DELETE'])
@require_login
def delete_state(state_id):
    state = SimulatorState.query.filter_by(id=state_id, user_id=current_user.id).first()
    if state:
        db.session.delete(state)
        db.session.commit()
        return jsonify({'success': True})
    return jsonify({'success': False, 'error': 'State not found'}), 404

def is_development_mode():
    replit_deployment = os.environ.get('REPLIT_DEPLOYMENT')
    if replit_deployment is None:
        replit_dev = os.environ.get('REPLIT_DEV_DOMAIN')
        return replit_dev is not None
    return replit_deployment != '1'

ALLOWED_TAGS = ['h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'p', 'strong', 'em', 'span', 
                'div', 'pre', 'code', 'br', 'a', 'ul', 'ol', 'li', 'i', 'b']
ALLOWED_ATTRS = {
    '*': ['class', 'id', 'title', 'data-section', 'data-tooltip'],
    'a': ['href'],
}
ALLOWED_PROTOCOLS = ['http', 'https', 'mailto']

def sanitize_html(html_content):
    return bleach.clean(
        html_content,
        tags=ALLOWED_TAGS,
        attributes=ALLOWED_ATTRS,
        protocols=ALLOWED_PROTOCOLS,
        strip=True
    )

@app.route('/api/environment')
def get_environment():
    return jsonify({
        'is_development': is_development_mode()
    })

@app.route('/api/landing-content', methods=['GET'])
def get_landing_content():
    contents = LandingPageContent.query.all()
    return jsonify({
        'contents': {c.section_key: c.content for c in contents}
    })

@app.route('/api/landing-content', methods=['POST'])
@require_login
def save_landing_content():
    if not is_development_mode():
        return jsonify({'success': False, 'error': 'Editing disabled in production'}), 403
    
    data = request.get_json()
    section_key = data.get('section_key')
    content = data.get('content')
    
    if not section_key or content is None:
        return jsonify({'success': False, 'error': 'Missing section_key or content'}), 400
    
    sanitized_content = sanitize_html(content)
    
    existing = LandingPageContent.query.filter_by(section_key=section_key).first()
    if existing:
        existing.content = sanitized_content
        existing.updated_by = current_user.id
    else:
        new_content = LandingPageContent(
            section_key=section_key,
            content=sanitized_content,
            updated_by=current_user.id
        )
        db.session.add(new_content)
    
    db.session.commit()
    return jsonify({'success': True})
