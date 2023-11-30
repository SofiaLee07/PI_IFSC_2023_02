#!/bin/env python3


"""
Carrega um modelo do Tensorflow Lite e faza algumas iterações com ele.

Os dados de entrada são formados por números aleatórios. O resultado da iteração não é
utilizado. O propósito desse script é apenas checar o tempo necessário para uma iteração.

Esse script utiliza a versão lite do modelo, que é o arquivo 'modelo.tflite'.

O modelo foi gerado com o código disponível no site: 'https://www.tensorflow.org/tutorials/images/segmentation'.
"""


import tensorflow.lite as tfl
import numpy as np
import time


# Carrega o modelo
interpretador = tfl.Interpreter(model_path="modelo.tflite", num_threads=1)

# Configuração básica do modelo, e informação da entrada e saída do modelo
interpretador.allocate_tensors()
entrada_info = interpretador.get_input_details()
saida_info = interpretador.get_output_details()


def testa_segmentacao():
    """
    Gera um conjunto de dados aleatórios e relaiza uma segmentação com o modelo.

    O tempo necessário para realizar a segmentação é apresentado no final dela.
    """
    # Entrada de dados gerada aleatóriamente
    formato_entrada = entrada_info[0]['shape']
    entrada = np.array(np.random.random_sample(formato_entrada), dtype=np.float32)

    t = time.time()  # Início da contagem

    # Define o tensor de entrada e realiza a segmentação. O resultado é posto a variável 'saida'
    interpretador.set_tensor(entrada_info[0]['index'], entrada)
    interpretador.invoke()
    saida = interpretador.get_tensor(saida_info[0]['index'])

    print("Tempo da segmentação: " + str(time.time()-t))  # Final da contagem


# Realiza o teste mais de uma vez. É normal a primeira vez ser mais lenta que as demais.
for loop in range(5):
    testa_segmentacao()
