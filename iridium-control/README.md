# Iridum email sender

This folder contains a Python 3 program for automatically sending Iridium command emails for controlling the HAB cutdown and vent.

## Setup

Make sure you're using Python 3 - this has only been tested on Python 3.11 running on Windows, YMMV.

### Dependencies

To install the required libraries, navigate to this directory and run `python3 -m pip install -r requirements.txt` on Linux, or `py -m pip install -r requirements.txt` on Windows. I recommend doing this in a venv, but it doesn't really matter.

### Environment variables

For security purposes, this repository doesn't contain the IMEI number for the Iridium modem. This must be added to a text file called `.env` in this directory. You must also add the email address that is being sent from to the `.env` file. It should look something like the following:
```
FROM=mynebpteam@gmail.com
IMEI=123456789123456
```

***DO NOT PUSH `.env` TO THE REPOSITORY***

### Iridium attachments

Place the `.sbd` file attachments in the `attachments/` directory. This directory will be created for you if it doesn't exist on first run.

### Gmail authentication credentials

To connect with the Gmail API, you need a `credentials.json` file for the account. Follow [these steps](https://developers.google.com/gmail/api/quickstart/python#authorize_credentials_for_a_desktop_application) to obtain the file.

## Running

Run `main.py` to start the program.
