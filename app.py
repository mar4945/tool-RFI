from flask import Flask, render_template, request, redirect, url_for
import os

app = Flask(__name__)

path_file = os.path.dirname(os.path.abspath(__file__))

# Define the main route for the home page
@app.route("/", methods=["GET", "POST"])
def home():
    if request.method == "POST":
        # Retrieve the input values from the form
        param1 = float(request.form.get("param1"))
        param2 = float(request.form.get("param2"))
        param3 = float(request.form.get("param3"))

        # Here you would start your simulation with these parameters
        # For example, you could call a function that uses these parameters:
        # start_simulation(param1, param2, param3)

        # After the simulation, redirect to the result page (optional)
        return redirect(url_for('result', param1=param1, param2=param2, param3=param3))

    # Render the HTML page with the form
    return render_template(path_file+"\_templates\home.html")

# Define a result route to display the results of the simulation (optional)
@app.route("/result")
def result():
    param1 = request.args.get("param1")
    param2 = request.args.get("param2")
    param3 = request.args.get("param3")
    # Add simulation result rendering here
    return f"Simulation started with values: Param1 = {param1}, Param2 = {param2}, Param3 = {param3}"

if __name__ == "__main__":
    app.run(debug=True)