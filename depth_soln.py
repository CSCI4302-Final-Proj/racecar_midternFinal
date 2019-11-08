#!/usr/bin/env python

import sys

def hello(variable):
	print(variable)
if __name__ == '__main__':
	data = sys.stdin.read()
	hello(data)
