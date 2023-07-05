from __future__ import print_function

import base64
import json
import os
from email.message import EmailMessage
from google.auth.transport.requests import Request
from google.oauth2.credentials import Credentials
from google_auth_oauthlib.flow import InstalledAppFlow
from googleapiclient.discovery import build
from googleapiclient.errors import HttpError


# If modifying these scopes, delete the file token.json.
SCOPES = ['https://mail.google.com/']

TOKEN_PATH = os.path.join('auth', 'token.json')
CREDS_PATH = os.path.join('auth', 'credentials.json')
ATTACHMENTS_PATH = 'attachments'
MESSAGES_PATH = 'messages'

ATTACHMENT_ACTIONS = {
    'idle': '000',
    'cutdown': '001',
    'vent open': '011',
    'vent close': '100'
}

class EmailSender:

    def __init__(self, imei, from_email):
        try:
            self.creds = self.authenticate_user()
        except HttpError as error:
            print(F'An error occurred in authentication: {error}')
            exit()
        # create gmail api client
        self.service = build('gmail', 'v1', credentials=self.creds)

        # generate and/or retrieve message request bodies
        msg_request_bodies = [self.retrieve_email_json(action_id, imei, from_email) for action_id in ATTACHMENT_ACTIONS.values()]
        # generate message sending lambdas
        msg_requests = [self.gmail_create_message_send_request(self.service, msg_request_body) for msg_request_body in msg_request_bodies]
        # create dictionary of {action: request()}
        self.action_requests = {action: request for action, request in zip(ATTACHMENT_ACTIONS.keys(), msg_requests)}

    def send_action(self, action: str):
        if self.action_requests[action] is not None:
            self.action_requests[action]()
        else:
            print(f'Error: {action} action API request not defined! Try restarting the program.')

    def authenticate_user(self):
        """Authenticates the user and creates a token.json file if needed.
        """
        creds = None
        # The file token.json stores the user's access and refresh tokens, and is
        # created automatically when the authorization flow completes for the first
        # time.
        if os.path.exists(TOKEN_PATH):
            creds = Credentials.from_authorized_user_file(TOKEN_PATH, SCOPES)
        # If there are no (valid) credentials available, let the user log in.
        if not creds or not creds.valid:
            if creds and creds.expired and creds.refresh_token:
                creds.refresh(Request())
            else:
                try:
                    flow = InstalledAppFlow.from_client_secrets_file(
                        CREDS_PATH, SCOPES)
                    creds = flow.run_local_server(port=0)
                except FileNotFoundError as error:
                    print(f'Make sure to add your credentials.json to the auth folder!')
                    if not os.path.exists('auth'):
                        os.mkdir('auth')
                    raise error
            # Save the credentials for the next run
            with open(TOKEN_PATH, 'w') as token:
                token.write(creds.to_json())

        if not creds:
            raise RuntimeError('authentication failed!')
        return creds


    def generate_email_json(self, attachment_filename: str, imei: str, from_: str,
                            to: str = 'data@sbd.iridium.com') -> str:
        '''
        Generate encoded email messages containing specified iridium attachment.
        Serialize the messages as JSON files and keep them for future runs.

        Args:
            attachment_filename (str): file name of the sbd attachment to use. ex: '000.sbd' 
            imei (str): IMEI number of the iridium modem.
            from_ (str): address to send the email from.
            to (str, optional): address to send the email to. Defaults to 'data@sbd.iridium.com'.

        Returns:
            str: name of the created .json file
        '''

        mime_message = EmailMessage()
        # headers
        mime_message['To'] = to
        mime_message['From'] = from_
        mime_message['Subject'] = imei
        # set body text to blank
        mime_message.set_content('')

        # add attachment
        attachment_path = os.path.join(ATTACHMENTS_PATH, attachment_filename)
        # set MIME type
        # content_type = 'application/octet-stream'
        # maintype, subtype = content_type.split('/', 1)
        maintype = 'application'
        subtype = 'octet-stream'

        with open(attachment_path, 'rb') as attachment_file:
            attachment_data = attachment_file.read()
        mime_message.add_attachment(attachment_data, maintype, subtype, filename=attachment_filename)
        # encode as base64
        encoded_message = base64.urlsafe_b64encode(mime_message.as_bytes()).decode()
        message_request_body = {
            'raw': encoded_message
        }
        # serialize encoded message as json and dump to a file
        json_file_name = os.path.splitext(attachment_filename)[0] + '.json'
        if not os.path.exists(MESSAGES_PATH):
            os.mkdir(MESSAGES_PATH)
        with open(os.path.join(MESSAGES_PATH, json_file_name), 'w') as json_file:
            json.dump(message_request_body, json_file)
        
        return json_file_name
        # service.users().messages().send(userId="me", body=message_request_body).execute()

    def retrieve_email_json(self, action_id: str, imei: str, from_: str) -> str:
        '''
        retrieves an email message from a corresponding json file.

        Args:
            action_id (str): id of the action performed by the email. json should be
                named {action_id}.json
            imei (str): IMEI number of the iridium modem.
            from_ (str): address to send the email from.

        Returns:
            str: the message request body for the email encoded in base64. 
        '''
        json_file_path = os.path.join(MESSAGES_PATH, action_id + '.json')
        message_request_body = ''
        try:
            with open(json_file_path, 'r') as json_file:
                message_request_body = json.load(json_file)
        except FileNotFoundError as error:
            print(f'JSON file for message with action id {action_id} not found! '
                    'Attempting to create it...')
            self.generate_email_json(action_id + '.sbd', imei, from_)

            print(f'Created file {action_id+".json"}. Retrieving data...')
            self.retrieve_email_json(action_id, imei, from_)

        return message_request_body

    def gmail_create_message_send_request(self, service, message_request_body: str):
        '''
        creates a lambda function for sending an email

        Args:
            service: gmail api service
            message_request_body (str): email message request body

        Returns:
            function: a lambda function that sends an email using the gmail API
        '''
        return lambda : service.users().messages().send(userId="me", body=message_request_body).execute()
