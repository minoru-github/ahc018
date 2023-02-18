# experiment

* Create "data" directory, and save input files to `./data`.  
* Create "out" directory.  
```
└─./experiment
    ├─./data
    └─./out
```
* Open `cmd` in `./experiment` directory.  
* Run commands below.  

```
python -m venv venv
```

```
activate-venv.bat
(venv) pip install mlflow
```

```
(venv) experiment.bat
```

```
(venv)mlflow ui
```