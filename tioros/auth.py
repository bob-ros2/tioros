#!/usr/bin/env python3
#
# Copyright 2023 BobRos
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

###############################################################################
# Simple Python Twitch authentication and token library.
# This library can optionaly also be used as cli tool.
# When using as lib import the function you wish to use. E.g.:
#
# from .auth import credentials_from_json_file, token_from_refresh_token
#
# Find below the cli help output:
#
# $ python3 auth.py -h
# usage: auth.py [-h] [-f CREDENTIALS] [-j] [-i ID] [-s SECRET] [-r REFRESH] [-c CODE] [-t STATE] [-p SCOPES] [-d REDIRECT]
# 
# Simple Twitch authentication and token cli tool. Can request token from refresh_token, authorization_code for given scopes, etc.
# 
# options:
#   -h, --help            show this help message and exit
#   -f CREDENTIALS, --credentials CREDENTIALS
#                         json file with credential data, will overwrite commandline args if given and contained in the file (default: )
#   -j, --json            dumps example credentials data (default: False)
#   -i ID, --id ID        client_id (default: )
#   -s SECRET, --secret SECRET
#                         client_secret (default: )
#   -r REFRESH, --refresh REFRESH
#                         refresh_token (default: )
#   -c CODE, --code CODE  code for authorization code grant flow, needs client_id and client_secret (default: )
#   -t STATE, --state STATE
#                         state, a unique id for the grant flow to prevent CSRF attacks, the server returns this string in the redirect URI and the client should prove it
#                         (default: )
#   -p SCOPES, --scopes SCOPES
#                         scopes, a whitespace delimeted list with scopes to be authorized, e.g.: moderator:read:followers channel:read:subscriptions chat:edit chat:read
#                         (default: )
#   -d REDIRECT, --redirect REDIRECT
#                         redirect_uri (default: http://localhost:3000)
# 

import os, sys, logging, requests, json, argparse, string, random
from urllib.parse import urlencode

# library

# allow request redirects
request_allow_redirects = False
# verifies SSL certificates if set to /path/to/certfile
request_verify = False

def do_post(
    client_id,
    client_secret   = None,
    grant_type      = 'client_credentials',
    redirect_uri    = None,
    refresh_token   = None,
    code            = None,
    auth_server_url = "https://id.twitch.tv/oauth2/token",
    allow_redirects = request_allow_redirects,
    verify          = request_verify):
    """
    Sends an oauth2 token POST request with given parameter.
    Returns a Python Dict with the response.
    """

    logging.captureWarnings(True)
    payload = {
        'grant_type': grant_type, 
        'client_id': client_id
    }

    if redirect_uri:  payload['redirect_uri'] = redirect_uri
    if client_secret: payload['client_secret'] = client_secret
    if refresh_token: payload['refresh_token'] = refresh_token
    if code:          payload['code'] = code

    response = requests.post(
        auth_server_url,
        data = payload, 
        verify = verify, 
        allow_redirects = allow_redirects,
        auth = (client_id, client_secret))

    logging.debug(str(response))

    if response.status_code !=200:
        logging.error(
            "Failed to obtain token from the OAuth2 server: "
            + str(response))
        return None

    return json.loads(response.text)


def token_from_client_credentials(
    client_id,
    client_secret):
    """
    Request token from given client_id and client_secret.
    Returns a Python Dict.
    """

    return do_post(
        client_id = client_id,
        client_secret = client_secret,
        grant_type = 'client_credentials')


def token_from_authorization_code(
    client_id,
    client_secret,
    code):
    """
    Request token from given client_id, client_secret and code.
    The code can be retrieved if you get the user (or yourself) to authorize your app
    using the Authorization code grant flow.
    Returns a Python Dict with token data.
    """

    return do_post(
        client_id = client_id,
        client_secret = client_secret,
        code = code,
        grant_type = 'authorization_code')


def token_from_refresh_token(
    client_id,
    client_secret,
    refresh_token):
    """
    Request token from given client_id, client_secret and refresh_token.
    The refresh_token will be retrieved e.g. if using the Authorization code 
    grant flow.
    The initial refresh_token will stay the same until the user removes the 
    connection associated with your client_id or change his password.
    Returns a Python Dict with token data.
    """

    return do_post(
        client_id = client_id,
        client_secret = client_secret,
        refresh_token = refresh_token,
        grant_type = 'refresh_token')


def credentials_from_json_file(
    path = None):
    """
    Load a file with JSON data into a Python Dict.
    If the path is None the function tries to load from ~/.credentials
    Returns a Python Dict with the content or None if loading from path fails.
    Prints a logging error message to the console if it fails.
    """

    if not path:
        path = os.path.join(
            os.path.expanduser('~'), '.credentials')

    try:
        with open(path, 'r') as f:
            return json.load(f)
    except Exception as e:
        logging.error(
            "credentials_from_json_file failed: "
            + str(e))

    return None


def code_from_client_id(
    client_id,
    scope = 'moderator:read:followers channel:read:subscriptions chat:edit chat:read',
    state = 'd3ab8b439efe11e793ae92361f002633',
    redirect_uri    = 'http://localhost:3000',
    auth_server_url = 'https://id.twitch.tv/oauth2/authorize'):
    """
    Produces an oauth2 authorize GET query url to get the user (or yourself) to authorize 
    your app with the Authorization code grant flow.
    This link has to be used in a browser to request the expected scope grants.
    The response will be the query url including the code needed in a next 
    step to get the token data.
    Returns a Python string with the query url.
    """

    query_string = urlencode({
        'client_id': client_id,
        'scope': scope,
        'state': state,
        'redirect_uri': redirect_uri,
        'response_type': 'code'
    })

    return f'{auth_server_url}?{query_string}'


def id_generator(size=32, chars=string.ascii_lowercase + string.digits):
    """
    Generate a random id in a given lengh with given characters.
    Returns a Python string with the generated id.
    """

    return ''.join(random.choice(chars) for _ in range(size))


# script entrypoint

def main():

    parser = argparse.ArgumentParser(
        description='Simple Twitch authentication and token cli tool. '
            'Can request token from refresh_token, '
            'authorization_code for given scopes, etc.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    
    parser.add_argument(
        '-f', '--credentials', 
        help = 'json file with credential data, will overwrite commandline args '
            'if given and contained in the file', 
        default = '')
    
    parser.add_argument(
        '-j', '--json', action="store_true", 
        help='dumps example credentials data')

    parser.add_argument(
        '-i', '--id', 
        help = 'client_id', 
        default = '')

    parser.add_argument(
        '-s', '--secret', 
        help = 'client_secret', 
        default = '')

    parser.add_argument(
        '-r', '--refresh', 
        help = 'refresh_token', 
        default = '')

    parser.add_argument(
        '-c', '--code', 
        help = 'code for authorization code grant flow, '
        'needs client_id and client_secret', 
        default = '')

    parser.add_argument(
        '-t', '--state', 
        help = 'state, a unique id for the grant flow to prevent CSRF attacks, '
        'the server returns this string in the redirect URI and the client should prove it', 
        default = '')

    parser.add_argument(
        '-p', '--scopes', 
        help = 'scopes, a whitespace delimeted list with scopes to be authorized, e.g.: '
        'moderator:read:followers channel:read:subscriptions chat:edit chat:read', 
        default = '')

    parser.add_argument(
        '-d', '--redirect', 
        help = 'redirect_uri', 
        default = 'http://localhost:3000')

    parsed, remaining = parser.parse_known_args()

    data = {
        "client_id": parsed.id, 
        "client_secret": parsed.secret, 
        "refresh_token": parsed.refresh,
        "code": parsed.code,
        "scopes": parsed.scopes,
        "redirect_uri": parsed.redirect,
        "state": id_generator()
    }

    if parsed.credentials:
        d = credentials_from_json_file(parsed.credentials)
        data.update(d)
    
    if parsed.json:
        print("Find below example credentials JSON data.\n"
            "Leave the entries empty which you don't need. E.g. for the "
            "refresh token flow just provide refresh_token, client_id, "
            "client_secret and state (optional)", file=sys.stderr)
        print(json.dumps(data))
        parser.exit(0)

    if data['refresh_token']:
        if not data['client_id']:
            print('client_id is missing')
            parser.exit(1)
        if not data['client_secret']:
            print('client_secret is missing')
            parser.exit(1)
        tdata = token_from_refresh_token(
            client_id = data['client_id'],
            client_secret = data['client_secret'],
            refresh_token = data['refresh_token'])
        print(json.dumps(tdata))

    elif data['code']:
        if not data['client_id']:
            print('client_id is missing')
            parser.exit(1)
        if not data['client_secret']:
            print('client_secret is missing')
            parser.exit(1)
        tdata = token_from_authorization_code(
            data['client_id'],
            data['client_secret'],
            data['code'])
        print(json.dumps(tdata))

    elif data['scopes']:
        if not data['client_id']:
            print('client_id is missing')
            parser.exit(1)
        if not data['redirect_uri']:
            print('redirect_uri is missing')
            parser.exit(1)
        redirect = code_from_client_id(
            client_id = data['client_id'],
            scope = data['scopes'],
            state = 'd3ab8b439efe11e793ae92361f002633',
            redirect_uri = data['redirect_uri'])
        print("Use below url in a browser to retrieve the authorization code\n"
            "Pass the code to this script 'code' option to get a token and "
            "refresh token", file=sys.stderr)
        print(redirect)
    
    elif data['client_id'] and data['client_secret']:
        tdata = token_from_client_credentials(
            data['client_id'],
            data['client_secret'])
        print(json.dumps(tdata))

    else: parser.print_help()


if __name__ == '__main__':
    main()
