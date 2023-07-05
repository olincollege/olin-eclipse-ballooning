from tkinter import *
from tkinter import ttk
from email_api import EmailSender

class EmailSenderGUI:

    def __init__(self, sender: EmailSender, root):
        self.sender = sender
        root.title("Iridium Email Sender")

        mainframe = ttk.Frame(root, padding="3 3 12 12")
        mainframe.grid(column=0, row=0, sticky=(N, W, E, S))
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)
        
        msgframe = ttk.Labelframe(mainframe, text="Send Iridium message:")
        msgframe.grid(column=0, row=0, padx=10, pady=10)
        ttk.Button(msgframe, text="Idle", command=lambda : self.sender.send_action('idle')).grid(column=0, row=0, sticky=(W))
        ttk.Button(msgframe, text="Cutdown", command=lambda : self.sender.send_action('cutdown')).grid(column=1, row=0, sticky=(E))
        ttk.Button(msgframe, text="Open Vent", command=lambda : self.sender.send_action('vent open')).grid(column=0, row=1, sticky=(W))
        ttk.Button(msgframe, text="Close Vent", command=lambda : self.sender.send_action('vent close')).grid(column=1, row=1, sticky=(E))

        for child in mainframe.winfo_children(): 
            child.grid_configure(padx=5, pady=5)
