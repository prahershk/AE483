# AE483: Final Project

## Controller
The controller used in the project is in the `controller_ae483.c` file.

## Jupyter Notebooks
The linearization is done in `linearization.ipynb`.

Offline testing was conducted in `offline_test.ipynb`. This contains all the graphs and results from the testing conducted.

## Python Files
When running the GUI.py in the conda environment, please run the following command in the terminal to integrate the PySimpleGUI library:
```
pip install PySimpleGUI
```

## Running the GUI
Following the install of PySimpleGUI, in order to launch the GUI that will show flight patterns, data parameters, and post-flight images, the following command must be run with the conda environment:
```
python GUI.py
```

## Pattern Photos
The following files are the search pattern photos that are used in the GUI:
- Square.png
- Sector.png
- Sector.png

## Flight Test Data
Flight data `.json` files are located in the `\flight-test-data` folder.

## Graphs
Graphs used in the report are located in the `\graphs` folder.

## Directory
```
project
|   controller_ae483.c
|   flight_gui.py
|   generate_results.py
|   GUI.py
|   linearization.ipynb
|   offline_test.ipynb
|   preflight_data.csv
|   README.md
|   Sector.png
|   Spiral.png
|   Square.png
|
└───flight-test-data
|   |   cc_co_hover_old_2.json
|   |   cc_co_hover_old_3.json
|   |   cc_co_hover_old.json
|   |   cc_co_square_old.json
|   |   cc_co_square_search.json
|   |   cc_do_hover.json
|   |   dc_do_hover.json
|
└───graphs
    |   dist_err.png
    |   ori_offline.png
    |   pos_offline.png
    |   results.png
    |   vel_offline.png
```
