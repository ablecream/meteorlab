import json
import pandas as pd
import matplotlib.pyplot as plt




def load_metadata(json_file):
    with open(json_file, 'r') as f:
        metadata = json.load(f)
    return metadata


data = load_metadata("../NASA_metadata.json")

apollo_metadata = data["APOLLO"]

print ("-"*40)

print("Number of Apollo samples:", len(apollo_metadata))

antaractic_metadata = data["ANTARCTIC"]

print("Number of Antarctic samples:", len(antaractic_metadata))


print("Total number of samples:", len(apollo_metadata) + len(antaractic_metadata))

print ("-"*40)



UniqueOrigins = set()

number_of_models_per_origin = {}

for sample in apollo_metadata:
    origin = sample.get("origin", "Unknown")
    UniqueOrigins.add(origin)
    number_of_models_per_origin[origin] = number_of_models_per_origin.get(origin, 0) + 1
    
for sample in antaractic_metadata:
    origin = sample.get("origin", "Unknown")
    UniqueOrigins.add(origin)
    number_of_models_per_origin[origin] = number_of_models_per_origin.get(origin, 0) + 1

print("Number of Unique Origins:", len(UniqueOrigins))

print("Number of models per origin:")
for origin, count in number_of_models_per_origin.items():
    print(f"- {origin}: {count} models")


print ("-"*40)



UniqueClassifications = set()

number_of_models_per_classification = {}

for sample in apollo_metadata:
    classification = sample.get("classification", "Unknown")
    UniqueClassifications.add(classification)
    number_of_models_per_classification[classification] = number_of_models_per_classification.get(classification, 0) + 1
for sample in antaractic_metadata:
    classification = sample.get("classification", "Unknown")
    UniqueClassifications.add(classification)
    number_of_models_per_classification[classification] = number_of_models_per_classification.get(classification, 0) + 1

print("Number of Unique Classifications:", len(UniqueClassifications))

print("Number of models per classification:")
for classification, count in number_of_models_per_classification.items():
    print(f"- {classification}: {count} models")