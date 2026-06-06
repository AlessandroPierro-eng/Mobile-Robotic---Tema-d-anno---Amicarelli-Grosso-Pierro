#!/bin/bash
echo "Ripristino la proprietà dei file all'utente locale ($USER)..."

# Sposta la proprietà ricorsivamente (-R) all'utente attuale per la cartella ros_ws
sudo chown -R $USER:$USER ros_ws

echo "Permessi sistemati! Ora puoi salvare su VS Code senza errori."