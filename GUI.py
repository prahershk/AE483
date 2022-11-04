import PySimpleGUI as sg
import os.path
import csv

# Theme
sg.theme('DarkAmber')

# Left side of GUI to create a search bar for the file
# Also displays the list of .png files in that folder
file_list_column = [
    [
        sg.Text("Desired Flight Plan")
    ],
    [
        sg.Listbox(
            values=['Lawnmower.png', 'SAR.png'], 
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
    [sg.Image(key="-IMAGE-")],
    [
        sg.Text('Drone Speed (m/s):'),
        sg.Input(key='-SPEED-', size=(5,1), do_not_clear=False)
    ],
    [
        sg.Text('X-limit (meters):'),
        sg.Input(key='-XLIM-', size=(5,1), do_not_clear=False),
        sg.Text('Y-limit (meters):'),
        sg.Input(key='-YLIM-', size=(5,1), do_not_clear=False)
    ],
    [
        sg.Button('Send to Drone'),
        sg.Text(size=(40, 1), key="-TOUT-")
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

    if event == 'Display':
        try:
            window["-PATH-"].update(values['-FILE LIST-'])
            # window["-IMAGE-"].update(values['-FILE LIST-'])
        except:
            pass

    if event == 'Send to Drone':
        # Takes data from the input and creates CSV file
        # Only occurs when button is pressed

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

        header = ['Speed', 'X Dimension', 'Y Dimension']
        data = [float(values['-SPEED-']), float(values['-XLIM-']), float(values['-YLIM-'])]

        with open('test_data', 'w', encoding='UTF8', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerow(data)

        window["-TOUT-"].update('SENT')

window.close()