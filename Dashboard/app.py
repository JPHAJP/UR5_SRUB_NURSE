from flask import Flask, render_template
import dash
import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Input, Output

app = Flask(__name__)

# Configuración de Dash dentro de Flask
dash_app = dash.Dash(__name__, server=app, url_base_pathname='/dashboard/')

# Estilo del layout del dashboard
dash_app.layout = html.Div(
    style={'backgroundColor': '#1F2430', 'color': '#FFFFFF'},
    children=[
        html.H1("Today's Sales Dashboard", style={'textAlign': 'center', 'color': '#FFFFFF'}),
        # Primer gráfico o componente
        dcc.Graph(
            id='example-graph',
            figure={
                'data': [
                    {'x': [1, 2, 3], 'y': [4, 1, 2], 'type': 'bar', 'name': 'Sales'},
                    {'x': [1, 2, 3], 'y': [2, 4, 5], 'type': 'bar', 'name': 'Visitors'},
                ],
                'layout': {
                    'title': 'Sales vs Visitors',
                    'plot_bgcolor': '#2C303A',
                    'paper_bgcolor': '#2C303A',
                    'font': {'color': '#FFFFFF'}
                }
            }
        ),
        # Más componentes o gráficos
        html.Div("Contenido adicional")
    ]
)

@app.route('/')
def home():
    return render_template('dashboard.html')

if __name__ == '__main__':
    app.run(debug=True)
