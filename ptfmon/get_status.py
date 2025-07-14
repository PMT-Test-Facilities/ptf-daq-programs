import requests
from bs4 import BeautifulSoup
import json

_obj = open("tokens.json")
data = json.load(_obj)
_obj.close()

protected_url = data["protected_url"]

headers = {
    'Accept': 'text/html,application/xhtml+xml,application/xml;q=0.9,image/avif,image/webp,image/apng,*/*;q=0.8,application/signed-exchange;v=b3;q=0.7',
    'Accept-Language': 'en-US,en;q=0.9',
    'Authorization': 'Basic YWRtbjpwbXRDYWxpYjEyMw==',
    'Cache-Control': 'max-age=0',
    'Connection': 'keep-alive',
    'Sec-Fetch-Dest': 'document',
    'Sec-Fetch-Mode': 'navigate',
    'Sec-Fetch-Site': 'none',
    'Sec-Fetch-User': '?1',
    'Upgrade-Insecure-Requests': '1',
    'User-Agent': 'Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/126.0.0.0 Safari/537.36',
    'sec-ch-ua': '"Not/A)Brand";v="8", "Chromium";v="126", "Google Chrome";v="126"',
    'sec-ch-ua-mobile': '?0',
    'sec-ch-ua-platform': '"macOS"',
}

def get_status():

    session = requests.Session()
    
    login_response = session.post(protected_url, headers=headers)
    soup = BeautifulSoup(login_response.content, 'html.parser')

    laser_row = soup.find('font', string=' Laser ')
    laser_status_td = laser_row.find_parent('td').find_next_sibling('td').find_next_sibling('td')
    laser_status = laser_status_td.find('font').get_text(strip=True)

    wiener_row = soup.find('font', string=' Wiener_Crate ')
    wiener_status_td = wiener_row.find_parent('td').find_next_sibling('td').find_next_sibling('td')
    wiener_status = wiener_status_td.find('font').get_text(strip=True)

    return laser_status, wiener_status
