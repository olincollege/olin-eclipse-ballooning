import os
from tkinter import Tk
from dotenv import load_dotenv
from gui import EmailSenderGUI
from email_api import EmailSender

if __name__ == '__main__':
    load_dotenv()
    from_email = os.environ.get('FROM')
    imei = os.environ.get('IMEI')
    if from_email is None or imei is None:
        raise NameError('Error: .env configuration not set! Make sure to '
                        'add the FROM email address and your IMEI to .env.')
    if not os.path.exists('attachments'):
        print('attachments not found! place .sbd attachments in the attachments/ directory.')
        os.mkdir('attachments')
    sender = EmailSender(imei, from_email)
    root = Tk()
    EmailSenderGUI(sender, root)
    root.mainloop()
