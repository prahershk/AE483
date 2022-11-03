import PySimpleGUI as sg
import os.path

# Left side of GUI to create a search bar for the file
# Also displays the list of .png files in that folder
file_list_column = [
    [
        sg.Text("Desired Flight Plan")
    ],
    [
        sg.Listbox(
            values=['Lawnmower.py'], enable_events=True, size=(40,20), key="-FILE LIST-"
        )
    ],
    [sg.Button('Display')]
]

# Right side of the GUI allows to view the .png selected 
# from the left side
image_viewer_column = [
    [
        sg.Text("Chosen flight path visualization:"),
        sg.Text(size=(40, 1), key="-TOUT-")
    ],
    [sg.Image(key="-IMAGE-")],
    [
        sg.Text('Enter speed here (m/s):'),
        sg.Input(key='-IN-', size=(10,1))
    ],
    [sg.Button('Send to Drone')]
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

# Will close the program if exited at any point
while True:
    event, values = window.read()
    if event == "Exit" or event == sg.WIN_CLOSED:
        break

    if event == 'Display':
        window["-TOUT-"].update(values['-FILE LIST-'])

    # File was instead chosen from the listbox
    if event == "-FILE LIST-":
        try:
            filename = os.path.join(
                values["-FOLDER-"], values["-FILE LIST-"][0]
            )
            window["-TOUT-"].update(filename)
            window["-IMAGE-"].update(filename=filename)
        except:
            pass

window.close()