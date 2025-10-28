#!/bin/bash

wc -l \
    ./{src,include}/*.{c,h}pp \
    ./{src,include}/*/*.{c,h}pp \
    ./{src,include}/*/*/*.{c,h}pp \
