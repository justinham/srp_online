
from flask import Flask, render_template, request, jsonify
import json
import math

import numpy as np
from scipy.interpolate import CubicHermiteSpline

from scipy.interpolate import CubicSpline, make_interp_spline

from pynmeagps import NMEAReader

from scipy.interpolate import UnivariateSpline

# import pandas as pd
import os
import logging



################ 

app = Flask(__name__)


@app.route('/')
def home():
#    return render_template('index.html')
    return render_template('test.html')



if __name__ == '__main__':
	# app.run(host='0.0.0.0', port=5001, debug=True, ssl_context=('cert.pem', 'key.pem'))
	app.run(host='0.0.0.0', port=5002, debug=True)

    
	