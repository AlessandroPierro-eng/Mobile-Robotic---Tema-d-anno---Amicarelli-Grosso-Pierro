#!/bin/bash
echo "Ripristino la proprietà dei file all'utente locale ($USER)..."

sudo chown -R $USER:$USER ros_ws

echo "Permessi sistemati! Ora puoi salvare su VS Code senza errori."