#ifndef DSU_H // Previne inclusões múltiplas
#define DSU_H

#include <vector> // Para usar std::vector

/**
 * @brief Implementação da estrutura de dados Disjoint Set Union (DSU) ou Union-Find.
 * Utilizada principalmente no algoritmo de Felzenszwalb para gerenciar os conjuntos (segmentos) de pixels.
 */
class DSU {
public:
    std::vector<int> parent;         // parent[i] armazena o pai do elemento i. Se parent[i] == i, i é a raiz do conjunto.
    std::vector<double> internal_diff; // Para Felzenszwalb: armazena a maior diferença de peso de aresta
                                     // dentro de um componente conectado (limiar interno).
    std::vector<int> size;           // Para Felzenszwalb: armazena o número de elementos (pixels) em um componente.

    /**
     * @brief Construtor da DSU. Inicializa 'n' conjuntos disjuntos, cada um com um elemento.
     * @param n O número total de elementos (pixels) a serem gerenciados.
     */
    DSU(int n);

    /**
     * @brief Encontra o representante (raiz) do conjunto ao qual o elemento 'i' pertence.
     * Implementa compressão de caminho para otimização.
     * @param i O índice do elemento.
     * @return O índice do representante do conjunto.
     */
    int find(int i);

    /**
     * @brief Tenta unir os conjuntos que contêm os elementos 'i' e 'j'.
     * Esta função é específica para o Felzenszwalb, usando peso da aresta e um limiar 'k'.
     * @param i O índice do primeiro elemento.
     * @param j O índice do segundo elemento.
     * @param w O peso da aresta (diferença entre i e j).
     * @param k O parâmetro de limiar para o Felzenszwalb.
     * @return true se os conjuntos foram unidos, false caso contrário (já eram o mesmo conjunto, ou o critério de união não foi satisfeito).
     */
    bool unite(int i, int j, double weight, double k);
};

#endif // DSU_H