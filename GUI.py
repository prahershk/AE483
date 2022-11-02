import PySimpleGUI as sg
import os.path

# Left side of GUI to create a search bar for the file
# Also displays the list of .png files in that folder
file_list_column = [
    [
        sg.Text("Desired Flight File")
    ],
    [
        sg.Listbox(
            values=[], enable_events=True, size=(40,20), key="-FILE LIST"
        )
    ],
]

# Right side of the GUI allows to view the .png selected 
# from the left side
image_viewer_column = [
    [sg.Text("Chosen flight path visualization:")],
    [sg.Text(size=(40, 1), key="-TOUT-")],
    [sg.Image(key="-IMAGE-")],
]

# Sets the general layout of the GUI
layout = [
    [
        sg.Column(file_list_column),
        sg.VSeparator(),
        sg.Column(image_viewer_column),
    ]
]

window = sg.Window("Search & Rescue Flight Paths", layout)

# Will close the program if exited at any point
while True:
    event, values = window.read()
    if event == "Exit" or event == sg.WIN_CLOSED:
        break

# Checks the folder name that was filled in and 
# lists all the .png files within
if event == "-FOLDER-":
    folder = values["-FOLDER-"]
    try:
        # List of files within the folder
        file_list = os.listdir(folder)
    except:
        file_list = []

    fnames = [
        f
        for f in file_list
        if os.path.isfile(os.path.join(folder,f))
        and f.lower().endswith((".png", ".gif"))
    ]
    window["-FILE LIST-"].update(fnames)

# File was instead chosen from the listbox
elif event == "-FILE LIST-":
    try:
        filename = os.path.join(
            values["-FOLDER-"], values["-FILE LIST-"][0]
        )
        window["-TOUT-"].update(filename)
        window["-IMAGE-"].update(filename=filename)
    except:
        pass

window.close()