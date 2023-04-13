from flask import Blueprint, request
from camera.obstacletr import ObstacleState

api = Blueprint('web', __name__)

@api.route('/obstacles/<Type>', methods = ['GET'])
def obstacles(Type):
    value = ObstacleState(Type)
    return value

@api.route('/obstacles', methods = ['GET'])
def obstacles_index():
    value = ObstacleState(None)
    return value

@api.route('/', methods = ['GET'])
def index():
    return obstacles_index()