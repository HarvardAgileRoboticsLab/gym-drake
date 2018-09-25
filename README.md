# gym-drake

Defines Drake Environments for use with OpenAI RL algorithms.

## Installation Instructions 

1) Checkout gym-drake: git clone https://github.com/HarvardAgileRoboticsLab/gym-drake.git

2) Make sure pip is installed (should be installed if using Python 2 >=2.7.9 or Python 3 >=3.4). If not, see [here](https://pip.pypa.io/en/stable/installing/). 

3) Install virtualenv: ```pip install --user virtualenv```

4) Setup virtual environment.

    a) ```cd gym-drake```
    
    b) ```virtualenv venv```

5) Activate venv (you have to do this for every terminal):  ```venv/bin/activate```. Simply use ```deactivate``` to deactivate the venv. 

6) Install tensorflow (see dependencies for details).

7) Install pydrake (see dependencies for details) using cmake. Don't forget to add pydrake to PYTHONPATH.



## Dependencies

### TensorFlow

Install from [pip](https://www.tensorflow.org/install/install_linux) or from [source](https://www.tensorflow.org/install/install_sources).

### OpenAI Gym

Install from pip or source [here](https://gym.openai.com/docs/#installation).

### pydrake
To install `pydrake` follow the [pydrake installation instructions](http://drake.mit.edu/python_bindings.html).




<!--- Install drake gym

2) Make sure pip is installed
3) Install virtualenv: pip install --user virtualenv
4) Setup virtual environment
  cd gym-drake
  virtualenv venv
5) Activate venv: venv/bin/activate (you have to do this for every terminal)
6) pip install tensorflow (or other versions)
7) Install pydrake using cmake (don't forget to add pydrake to PYTHONPATH) --->
