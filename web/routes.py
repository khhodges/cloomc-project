from flask import session, jsonify, request, send_from_directory
from app import app, db
from replit_auth import require_login, make_replit_blueprint
from flask_login import current_user
from models import SimulatorState
import json
import os
import logging

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
