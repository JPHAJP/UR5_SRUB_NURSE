from flask import Flask, render_template

app = Flask(__name__)

@app.route('/')
def home():
    valores = {
        'ur_x': 0, 'ur_y': 0, 'ur_z': 0, 
        'ur_rx': 0, 'ur_ry': 0, 'ur_rz': 0, # ur son las coordenadas del robot
        'x': 0, 'y': 0, 'z': 0,             # x, y, z son las coordenadas del objeto
        'rx': 0, 'ry': 0, 'rz': 0
    }
    
    return render_template('index.html', **valores)

if __name__ == '__main__':
    app.run(debug=True)