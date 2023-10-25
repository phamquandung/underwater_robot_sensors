#!/usr/bin/env python3

import requests
from termcolor import colored

class GPSInteraface:
    def __init__(self, url):
        self.url = url
    
    # WaterLinked API methods
    def get_data(self, url):
        try:
            r = requests.get(url)
        except requests.exceptions.RequestException as exc:
            print(colored("Exception occured {}".format(exc), "red"))
            return None

        if r.status_code != requests.codes.ok:
            print("Got error {}: {}. Problems getting relative/global Locator position from the topside".format(r.status_code, r.text))
            return None
        return r.json()
    
    def get_acoustic_position(self):
        print(f"Getting acoustic position from {self.url}")
        return self.get_data("{}/api/v1/position/acoustic/filtered".format(self.url))
    
    def get_global_position(self):
        return self.get_data("{}/api/v1/position/global".format(self.url))