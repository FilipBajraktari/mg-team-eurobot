import urllib.request
import json

url_address = 'http://127.0.0.1:8080/obstacles'

def fetch():
    webUrl = urllib.request.urlopen(url=url_address)
    print(f'result code: {str(webUrl.getcode())}')
    data = webUrl.read()
    print(data)
    print(f'Parsed result: {json.loads(data)}')

fetch()