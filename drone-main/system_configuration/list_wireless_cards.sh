#!/bin/sh

iw dev | awk '$1=="Interface"{print $2}'
