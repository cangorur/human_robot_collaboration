#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 24 23:45:29 2020

@author: cangorur
"""

import sklearn

if __name__=='__main__':
    score = sklearn.metrics.mutual_info_score([0,1],[1,0])
    print(score)
    