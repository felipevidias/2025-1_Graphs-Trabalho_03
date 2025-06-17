#include "DSU.h"     // Inclui o cabeçalho da própria classe DSU
#include <algorithm> // Necessário para std::min e std::max

/**
 * @brief Construtor da classe DSU. Inicializa 'n' conjuntos disjuntos.
 * Cada elemento é inicialmente seu próprio pai, tem uma diferença interna de 0,
 * e um tamanho de 1 (pois é o único membro do seu conjunto).
 * @param n O número total de elementos (pixels) que serão gerenciados pela DSU.
 */
DSU::DSU(int n) {
    parent.resize(n);         // Redimensiona o vetor 'parent' para 'n' elementos.
    internal_diff.resize(n, 0.0); // Redimensiona 'internal_diff' para 'n' elementos, inicializando com 0.0.
    size.resize(n, 1);        // Redimensiona 'size' para 'n' elementos, inicializando com 1.
    // Cada elemento é inicialmente um conjunto independente, sendo seu próprio pai.
    for (int i = 0; i < n; ++i) parent[i] = i;
}

/**
 * @brief Encontra o representante (raiz) do conjunto ao qual o elemento 'i' pertence.
 * Implementa a otimização de compressão de caminho (path compression),
 * que achata a árvore do conjunto, tornando futuras operações 'find' mais rápidas.
 * @param i O índice do elemento cujo representante está sendo procurado.
 * @return O índice do representante (raiz) do conjunto.
 */
int DSU::find(int i) {
    // Se o elemento 'i' não é o seu próprio pai, ele não é a raiz.
    if (parent[i] != i) {
        // Recursivamente encontra a raiz e, no caminho de volta,
        // faz com que 'i' aponte diretamente para a raiz (compressão de caminho).
        parent[i] = find(parent[i]);
    }
    return parent[i]; // Retorna a raiz do conjunto.
}

/**
 * @brief Tenta unir os conjuntos que contêm os elementos 'i' e 'j'.
 * Esta função é a parte central do algoritmo de Felzenszwalb.
 * Ela une dois componentes (conjuntos) apenas se a aresta 'w' entre eles
 * não for muito maior do que a menor "variação interna" dos dois componentes.
 * @param i O índice do primeiro elemento.
 * @param j O índice do segundo elemento.
 * @param w O peso da aresta que conecta 'i' e 'j' (diferença entre os pixels).
 * @param k O parâmetro de limiar do Felzenszwalb, que controla a granularidade da segmentação.
 * @return true se os conjuntos foram unidos com sucesso, false caso contrário.
 */
bool DSU::unite(int i, int j, double w, double k) {
    int ri = find(i); // Encontra a raiz do conjunto de 'i'.
    int rj = find(j); // Encontra a raiz do conjunto de 'j'.

    // Se os elementos 'i' e 'j' já estão no mesmo conjunto, não há nada a unir.
    if (ri == rj) return false;

    // Calcula o limiar de variação interna para cada componente.
    // Mi = maior diferença de aresta dentro do componente ri + k / tamanho do componente ri.
    // Mj = maior diferença de aresta dentro do componente rj + k / tamanho do componente rj.
    // O 'k / size' permite que componentes menores (com size pequeno) aceitem apenas arestas com w menor,
    // enquanto componentes maiores (com size grande) podem aceitar arestas com w maiores.
    double Mi = internal_diff[ri] + k / size[ri];
    double Mj = internal_diff[rj] + k / size[rj];

    // Critério de união do Felzenszwalb:
    // Se o peso da aresta 'w' for maior do que a menor das variações internas (Mi ou Mj),
    // significa que a aresta 'w' é muito diferente de ambos os componentes, e eles não devem ser unidos.
    if (w > std::min(Mi, Mj)) return false;

    // Otimização de união por tamanho (union by size):
    // Sempre anexa a árvore menor à raiz da árvore maior para manter as árvores achatadas.
    if (size[ri] < size[rj]) std::swap(ri, rj); // Garante que ri é a raiz do conjunto maior ou igual.

    parent[rj] = ri;     // Faz a raiz do conjunto menor (rj) apontar para a raiz do conjunto maior (ri).
    size[ri] += size[rj]; // Atualiza o tamanho do novo conjunto combinado.

    // Atualiza a maior diferença interna do novo componente combinado.
    // É o máximo entre as diferenças internas originais dos dois componentes e o peso da aresta 'w' que os uniu.
    internal_diff[ri] = std::max({internal_diff[ri], internal_diff[rj], w});
    
    return true; // A união foi bem-sucedida.
}