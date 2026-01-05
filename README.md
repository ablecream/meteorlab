# Pipeline de Traitement de Modèles 3D

Ce projet contient une pipeline pour traiter des modèles 3D, incluant une simplification de maillage basée sur la courbure et une segmentation par régions.

## Structure du Projet

- `adaptive_decimation/`: Projet C++ pour la simplification de maillage.
- `segmentation/`: Projet C++ pour la segmentation des maillages.
- `NASA_obj/`: Dossier d'entrée pour les modèles 3D au format `.obj`.
- `run_pipeline.sh`: Script principal pour orchestrer l'ensemble de la pipeline.

## Prérequis

Pour compiler et exécuter ce projet, vous aurez besoin de :
- `cmake`
- Un compilateur C++ (ex: `g++`)
- Les bibliothèques `PCL` (Point Cloud Library) et `VTK`, incluant leurs dépendances.

## Instructions

### 1. Compilation

Avant la première utilisation, vous devez compiler les deux projets C++.

```bash
# Boucle pour compiler les deux projets C++
for project in adaptive_decimation segmentation; do
    echo "--- Compilation de $project ---"
    
    # Crée un dossier de build et s'y déplace
    mkdir -p "$project/build"
    cd "$project/build"
    
    # Exécute cmake et make
    cmake ..
    make
    
    # Retourne à la racine du projet
    cd ../..
done

echo "--- Compilation terminée ---"
```

### 2. Données d'Entrée

Placez tous vos modèles 3D au format `.obj` dans le dossier `NASA_obj/`.

### 3. Exécution

Pour lancer la pipeline de traitement sur tous les modèles du dossier d'entrée, exécutez le script principal :

```bash
./run_pipeline.sh
```

## 4. Sorties

Les résultats seront générés dans le dossier `pipeline_output/`, structuré comme suit :

- `pipeline_output/1_simplified_ply/`: Contient les modèles simplifiés au format `.ply`.
- `pipeline_output/2_segmented_labels/`: Contient les fichiers texte avec les labels de chaque face.
- `pipeline_output/3_segmented_visual/`: Contient les maillages avec une coloration par segment pour la visualisation.
