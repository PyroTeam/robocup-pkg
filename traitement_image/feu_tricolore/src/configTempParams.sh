#!/bin/bash
## Script permettant d'instancier des paramètres de test pour le traitement d'images
rosparam set /trait_im/feu_tricolore/tmp/ "
H: 
  threshold: {enabled: false, min: 150, max: 255}
  morphops:
    opening: {enabled: false, iterations: 1, size: 1}
    closing: {enabled: false, iterations: 1, size: 1}
S: 
  threshold: {enabled: false, min: 150, max: 255}
  morphops:
    opening: {enabled: false, iterations: 1, size: 1}
    closing: {enabled: false, iterations: 1, size: 1}
V: 
  threshold: {enabled: true, min: 220, max: 255}
  morphops:
    opening: {enabled: true, iterations: 1, size: 1}
    closing: {enabled: true, iterations: 1, size: 5}
R: 
  threshold: {enabled: true, min: 150, max: 255}
  morphops:
    opening: {enabled: false, iterations: 1, size: 1}
    closing: {enabled: false, iterations: 1, size: 1}
G: 
  threshold: {enabled: true, min: 200, max: 255}
  morphops:
    opening: {enabled: true, iterations: 1, size: 1}
    closing: {enabled: true, iterations: 1, size: 5}
B: 
  threshold: {enabled: true, min: 150, max: 255}
  morphops:
    opening: {enabled: false, iterations: 1, size: 1}
    closing: {enabled: false, iterations: 1, size: 1}"