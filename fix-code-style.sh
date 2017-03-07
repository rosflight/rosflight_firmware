#!/bin/bash

CMD="astyle --options=.astylerc"

$CMD include/*
$CMD --exclude="param.c" src/*
$CMD lib/turbotrig/*
