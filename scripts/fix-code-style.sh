#!/bin/bash

CMD="astyle --options=.astylerc"

$CMD ../include/*
$CMD ../src/*
$CMD ../lib/turbotrig/*
$CMD ../boards/F1/*
$CMD ../boards/F4/*
