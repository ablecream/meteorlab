import json
import os
from fpdf import FPDF

# --- FONCTION DE CHARGEMENT ---
def load_metadata(json_file):
    with open(json_file, 'r', encoding='utf-8') as f:
        metadata = json.load(f)
    return metadata

# --- CLASSE PDF PERSONNALISÉE ---
class PDFReport(FPDF):
    def header(self):
        # En-tête du document (Couleur Bleu NASA)
        self.set_font("helvetica", "B", 18)
        self.set_text_color(11, 61, 145) 
        self.cell(0, 10, "Rapport d'Analyse des Métadonnées - Météorites", align="C", ln=True)
        self.ln(10)

    def footer(self):
        # Pied de page
        self.set_y(-15)
        self.set_font("helvetica", "I", 9)
        self.set_text_color(128, 128, 128)
        self.cell(0, 10, f"Page {self.page_no()}/{2}", align="C")

    def section_title(self, title):
        # Style des titres de section
        self.set_font("helvetica", "B", 14)
        self.set_text_color(40, 40, 40)
        self.cell(0, 10, title, ln=True)
        # Ligne de séparation
        self.set_draw_color(200, 200, 200)
        self.line(self.get_x(), self.get_y(), self.get_x() + 190, self.get_y())
        self.ln(5)

    def print_stat_row(self, label, value, is_bold_value=True):
        # Affichage d'une ligne de statistique
        self.set_font("helvetica", "", 11)
        self.set_text_color(0, 0, 0)
        self.cell(140, 7, label, ln=0)
        
        if is_bold_value:
            self.set_font("helvetica", "B", 11)
            self.set_text_color(11, 61, 145)
            
        self.cell(0, 7, str(value), ln=True)

def main():
    # 1. Chargement des données
    data = load_metadata("../NASA_metadata2.json")
    apollo_metadata = data.get("APOLLO", [])
    antarctic_metadata = data.get("ANTARCTIC", [])

    # 2. Traitement des données
    number_of_models_per_origin = {}
    number_of_models_per_classification = {}

    for sample in apollo_metadata + antarctic_metadata:
        # Origines
        origin = sample.get("origin", "Inconnue")
        number_of_models_per_origin[origin] = number_of_models_per_origin.get(origin, 0) + 1
        
        # Classifications
        classification = sample.get("macro_category", "Inconnue")
        number_of_models_per_classification[classification] = number_of_models_per_classification.get(classification, 0) + 1

    # Tri des dictionnaires par valeur décroissante pour une meilleure présentation
    sorted_origins = dict(sorted(number_of_models_per_origin.items(), key=lambda item: item[1], reverse=True))
    sorted_classifications = dict(sorted(number_of_models_per_classification.items(), key=lambda item: item[1], reverse=True))

    # 3. Création du PDF
    pdf = PDFReport()
    pdf.add_page()

    # --- SECTION 1 : Vue d'ensemble ---
    pdf.section_title("1. Vue d'Ensemble des Échantillons")
    pdf.print_stat_row("Échantillons de la collection Apollo :", f"{len(apollo_metadata)} modèles")
    pdf.print_stat_row("Échantillons de la collection Antarctique :", f"{len(antarctic_metadata)} modèles")
    pdf.ln(2)
    pdf.set_font("helvetica", "B", 12)
    pdf.print_stat_row("Nombre total d'échantillons traités :", f"{len(apollo_metadata) + len(antarctic_metadata)} modèles")
    pdf.ln(10)

    # --- SECTION 2 : Origines ---
    pdf.section_title(f"2. Répartition par Origine ({len(sorted_origins)} origines uniques)")
    for origin, count in sorted_origins.items():
        # Utiliser un style de puce visuelle
        pdf.print_stat_row(f"    - {origin}", f"{count} modèles", is_bold_value=False)
    pdf.ln(10)

    # --- SECTION 3 : Classifications ---
    # Ajout d'une nouvelle page si on manque de place
    if pdf.get_y() > 200:
        pdf.add_page()
        
    pdf.section_title(f"3. Répartition par Classification ({len(sorted_classifications)} classifications uniques)")
    for classification, count in sorted_classifications.items():
        pdf.print_stat_row(f"    - {classification}", f"{count} modèles", is_bold_value=False)

    # 4. Sauvegarde du fichier
    output_filename = "Rapport_Metadonnees_NASA.pdf"
    pdf.output(output_filename)
    print(f"Succès ! Le rapport a été généré et sauvegardé sous le nom : {output_filename}")

if __name__ == "__main__":
    main()