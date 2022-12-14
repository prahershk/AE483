import PySimpleGUI as sg
import csv
import matplotlib.pyplot as plt
import random

### This python file creates the Graphical User Interface (GUI) and allows
### the user to pick a pre-determined flight path for the drone to fly.
### Following this, the user can display, pick drone speed and x&y limits
### send to drone. The final flight path is relayed back to the display.

# Randomly picks a theme for the GUI
theme_name_list = sg.theme_list()
random_theme = random.choice(theme_name_list)
sg.theme(random_theme)

# Left side of GUI to display the list of available Search Patterns
file_list_column = [
    [
        sg.Text("Desired Flight Plan")
    ],
    [
        sg.Listbox(
            values=[
                'Square Pattern Single-Unit',
                'Parallel Single-Unit Spiral',
                'Sector Pattern Single-Unit'
            ],
            enable_events=True,
            size=(40,20),
            key="-FILE LIST-"
        )
    ],
    [sg.Button('Display')]
]

# Right side of the GUI allows to view the .png selected 
# from the left side
image_viewer_column = [
    [
        sg.Text("Chosen flight path visualization:"),
        sg.Text(size=(40,1), key='-PATH-')
    ],
    [sg.Image(key="-IMAGE-", size=(246,164))],
    [
        sg.Text('Drone Speed (m/s):'),
        sg.Input(key='-SPEED-', size=(5,1), do_not_clear=False)
    ],
    [
        sg.Text('X-limit (m):'),
        sg.Input(key='-XLIM-', size=(5,1), do_not_clear=False),
        sg.Text('Y-limit (m):'),
        sg.Input(key='-YLIM-', size=(5,1), do_not_clear=False)
    ],
    [
        sg.Button('Send to Drone'),
        sg.Text(size=(40, 1), key="-TOUT-")
    ],
    [
        sg.Button('Fly Drone'),
        sg.Text(size=(40,1), key="-FLY-")
    ],
    [
        sg.Button('Show Actual Flight'),
        sg.Text(size=(40,1), key='-SHOW-')
    ]
]

# Sets the general layout of the GUI
layout = [
    [
        sg.Column(file_list_column),
        sg.VSeparator(),
        sg.Column(image_viewer_column)
    ]
]

window = sg.Window("Search & Rescue Flight Paths", layout)

# Processing User Input
while True:
    event, values = window.read()
    if event == "Exit" or event == sg.WIN_CLOSED:
        break

    # Displays the flight pattern that is selected in the window
    if event == 'Display':
        try:
            if values['-FILE LIST-'] == ['Square Pattern Single-Unit']:
                filename = 'Square.png'
            elif values['-FILE LIST-'] == ['Parallel Single-Unit Spiral']:
                filename = 'Spiral.png'
            elif values['-FILE LIST-'] == ['Sector Pattern Single-Unit']:
                filename = 'Sector.png'

            window["-PATH-"].update(values['-FILE LIST-'])
            window['-IMAGE-'].update(filename=filename)
        except:
            window["-PATH-"].update("That shit don't exist!!")

    # Takes data from the input and creates CSV file
    if event == 'Send to Drone':
        # Limit corrections if over/under bounds
        lower = 0.1
        upper = 2.0
        if float(values['-SPEED-']) <= lower:
            values['-SPEED-'] = lower
        
        if float(values['-SPEED-']) >= upper:
            values['-SPEED-'] = upper

        if values['-XLIM-'] == '':
            values['-XLIM-'] = 0.0

        if values['-YLIM-'] == '':
            values['-YLIM-'] = 0.0
        
        # Header/data written to the CSV file
        header = ['Speed', 'X Dimension', 'Y Dimension', 'Flight Pattern']
        data = [float(values['-SPEED-']), float(values['-XLIM-']), float(values['-YLIM-']), values['-FILE LIST-']]

        with open('preflight_data.csv', 'w', encoding='UTF8', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerow(data)

        window["-TOUT-"].update('SENT')

    # Runs the flight_gui.py file that will fly the drone
    if event == 'Fly Drone':
        file = 'flight_gui.py'
        with open(file) as infile:
            exec(infile.read())

    # Once flight is done, the actual flight path can be viewed
    if event == 'Show Actual Flight':
        flight_file = 'generate_results.py'
        with open(flight_file) as infile:
            exec(infile.read())

        filename = 'results.png'
        window['-IMAGE-'].update(filename=filename)

window.close()