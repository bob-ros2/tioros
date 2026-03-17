#
# Copyright 2023 Bob Ros
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

import argparse
import json
import logging
import os
import random
import string
import sys
from urllib.parse import urlencode

import requests

# library

# allow request redirects
request_allow_redirects = False
# verifies SSL certificates if set to /path/to/certfile
request_verify = False


def do_post(
    client_id,
    client_secret=None,
    grant_type='client_credentials',
    redirect_uri=None,
    refresh_token=None,
    code=None,
    auth_server_url="https://id.twitch.tv/oauth2/token",
    allow_redirects=request_allow_redirects,
    verify=request_verify
):
    """
    Send an oauth2 token POST request with given parameters.

    Returns a Python Dict with the response.
    """
    logging.captureWarnings(True)
    payload = {
        'grant_type': grant_type,
        'client_id': client_id
    }

    if redirect_uri:
        payload['redirect_uri'] = redirect_uri
    if client_secret:
        payload['client_secret'] = client_secret
    if refresh_token:
        payload['refresh_token'] = refresh_token
    if code:
        payload['code'] = code

    response = requests.post(
        auth_server_url,
        data=payload,
        verify=verify,
        allow_redirects=allow_redirects,
        auth=(client_id, client_secret))

    logging.debug(str(response))

    if response.status_code != 200:
        logging.error(
            "Failed to obtain token from the OAuth2 server: "
            + str(response))
        return None

    return json.loads(response.text)


def token_from_client_credentials(client_id, client_secret):
    """Request token from given client_id and client_secret."""
    return do_post(
        client_id=client_id,
        client_secret=client_secret,
        grant_type='client_credentials')


def token_from_authorization_code(client_id, client_secret, code):
    """Request token from given client_id, client_secret and code."""
    return do_post(
        client_id=client_id,
        client_secret=client_secret,
        code=code,
        grant_type='authorization_code')


def token_from_refresh_token(client_id, client_secret, refresh_token):
    """Request token from given client_id, client_secret and refresh_token."""
    return do_post(
        client_id=client_id,
        client_secret=client_secret,
        refresh_token=refresh_token,
        grant_type='refresh_token')


def credentials_from_json_file(path=None):
    """Load a file with JSON data into a Python Dict."""
    if not path:
        path = os.path.join(os.path.expanduser('~'), '.credentials')

    try:
        with open(path, 'r') as f:
            return json.load(f)
    except Exception as e:
        logging.error("credentials_from_json_file failed: " + str(e))

    return None


def code_from_client_id(
    client_id,
    scope='moderator:read:followers channel:read:subscriptions chat:edit chat:read',
    state='d3ab8b439efe11e793ae92361f002633',
    redirect_uri='http://localhost:3000',
    auth_server_url='https://id.twitch.tv/oauth2/authorize'
):
    """Produce an oauth2 authorize GET query url."""
    query_string = urlencode({
        'client_id': client_id,
        'scope': scope,
        'state': state,
        'redirect_uri': redirect_uri,
        'response_type': 'code'
    })

    return f'{auth_server_url}?{query_string}'


def id_generator(size=32, chars=string.ascii_lowercase + string.digits):
    """Generate a random id."""
    return ''.join(random.choice(chars) for _ in range(size))


def main():
    """Script entrypoint."""
    parser = argparse.ArgumentParser(
        description='Simple Twitch authentication and token cli tool.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument(
        '-f', '--credentials',
        help='json file with credential data',
        default='')

    parser.add_argument(
        '-j', '--json', action="store_true",
        help='dumps example credentials data')

    parser.add_argument('-i', '--id', help='client_id', default='')
    parser.add_argument('-s', '--secret', help='client_secret', default='')
    parser.add_argument('-r', '--refresh', help='refresh_token', default='')
    parser.add_argument('-c', '--code', help='authorization code', default='')
    parser.add_argument('-t', '--state', help='state (CSRF protection)', default='')
    parser.add_argument('-p', '--scopes', help='whitespace delimited scopes', default='')
    parser.add_argument('-d', '--redirect', help='redirect_uri', default='http://localhost:3000')

    parsed, _ = parser.parse_known_args()

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
        if d:
            data.update(d)

    if parsed.json:
        print("Find below example credentials JSON data.\n"
              "Leave the entries empty which you don't need.", file=sys.stderr)
        print(json.dumps(data))
        parser.exit(0)

    if data['refresh_token']:
        if not data['client_id'] or not data['client_secret']:
            print('client_id or client_secret is missing')
            parser.exit(1)
        tdata = token_from_refresh_token(
            client_id=data['client_id'],
            client_secret=data['client_secret'],
            refresh_token=data['refresh_token'])
        print(json.dumps(tdata))

    elif data['code']:
        if not data['client_id'] or not data['client_secret']:
            print('client_id or client_secret is missing')
            parser.exit(1)
        tdata = token_from_authorization_code(
            data['client_id'],
            data['client_secret'],
            data['code'])
        print(json.dumps(tdata))

    elif data['scopes']:
        if not data['client_id'] or not data['redirect_uri']:
            print('client_id or redirect_uri is missing')
            parser.exit(1)
        redirect = code_from_client_id(
            client_id=data['client_id'],
            scope=data['scopes'],
            state='d3ab8b439efe11e793ae92361f002633',
            redirect_uri=data['redirect_uri'])
        print("Use below url in a browser to retrieve the authorization code", file=sys.stderr)
        print(redirect)

    elif data['client_id'] and data['client_secret']:
        tdata = token_from_client_credentials(
            data['client_id'],
            data['client_secret'])
        print(json.dumps(tdata))

    else:
        parser.print_help()


if __name__ == '__main__':
    main()
