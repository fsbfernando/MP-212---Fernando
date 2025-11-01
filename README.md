A cena correspondente para usar em conjunto com esses scripts está em:
https://drive.google.com/file/d/1v1-0LIL-osAGh9eCEU1aAADdXwm7zBXw/view?usp=drive_link
arquivo: v1_trab_bi_212.ttt

Para funcionar, é necessário usar ROS.
Após o 'roscore' e play na simulação, pode-se controlar o AMR e o KUKA em seus respectivos códigos.

A realsense está modelada e presente no coppelia e pode ser usada mediante uso do código de controle via MATLAB disponível nesse repositório.

Devido a alta quantidade de dados, é necessário o uso de GPU para rodar a simulação

yolorunningv4 -> não existe filtro para pegar apenas uma região 'junta do bagageiro', trata a nuvem de ponto, mas pega todo o bagageiro.

yolorunningv5 -> filtra apenas a maior região conexa do bagageiro na nuvem de pontos

yolorunningv6 -> versão 1/11/25

Fernando-Braga/runs/detect/train12/weights   -> diretório pc salvo com os dados do treinamento best_v1.pt
