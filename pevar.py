# get an access token via OAUTH, then use it to request latest d2w reading
# works with p5c

import requests
from requests.auth import HTTPBasicAuth


# grab token
url = 'https://api.particle.io/oauth/token'

headers = {'Content-type':'application/x-www-form-urlencoded',
           'Authorization':'particle:particle'}
arguments = {'grant_type':'password',
             'username':'meshlab@soest.hawaii.edu',
             'password':'BBB!ocnData3'
             }

r = requests.post(url,
                  headers=headers,
                  auth=HTTPBasicAuth('particle','particle'),
                  data=arguments)
print(r.json())
token = r.json()['access_token']
#token = '70061b940d98bc092b5c1c14c46eca6ed0825cc7'


# request latest d2w reading
coreid = '280021001951353338363036' # 045
#coreid = '410055001951353338363036' # 047
#coreid = '4e0029001751353338363036' # 052
#coreid = '40002e001951353338363036' # 058
#coreid = '4b002f001951343334363036' # 070
#coreid = '40004f001951353338363036' # 076
url = 'https://api.particle.io/v1/devices/{}/d2w?access_token={}'.format(coreid,token)

print
print(url)

print
for i in range(10):
    r = requests.get(url)
    print(r.text)
