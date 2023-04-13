from flask import Flask, jsonify
from application.blueprints.routes import api

app = Flask(__name__)
app.register_blueprint(api, url_prefix='/')

@app.errorhandler(404)
def not_found(error):
    return jsonify({'error': 'Not Found'}), 404

@app.errorhandler(403)
def forbidden(error):
    return jsonify({'error': 'Not Found'}), 403

@app.errorhandler(404)
def bad_request(error):
    return jsonify({'error': 'Not Found'}), 402