# from flask import Flask, render_template, request, jsonify
# from application.utility import get_config_JSON
# import json

# from sim_controller import simulation

# app = Flask(__name__)

# @app.route("/")
# def hello_world():
#     return render_template("ago.html")

# @app.route("/run_simulation", methods = ["GET","POST"])
# def run_simulation():
    
#     config = {}
#     config["nu"] = request.form["param1"]
#     config["nx"] = request.form["param2"]
    
#     result = get_config_JSON(config)
    
#      = simulation(result)
    
#     return jsonify(result)

# if __name__ == "__main__":
#     app.run(debug=True)