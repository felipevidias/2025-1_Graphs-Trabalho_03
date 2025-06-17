#ifndef IMAGE_SEGMENTER_H // Previne inclusões múltiplas
#define IMAGE_SEGMENTER_H

#include "Image.h" // Inclui a classe Image
#include "DSU.h"   // Inclui a classe DSU (Disjoint Set Union)
#include <vector>  // Para usar std::vector
#include <queue>   // Para usar std::priority_queue (para IFT)

/**
 * @brief Estrutura que representa uma aresta em um grafo.
 * Contém os índices dos vértices (pixels) que conecta e o peso da aresta.
 */
struct Edge {
    int u, v;       // Índices dos dois vértices (pixels) conectados pela aresta
    double weight;  // Peso da aresta (geralmente baseado na diferença entre os pixels)
};

/**
 * @brief Classe responsável pela segmentação de imagens usando Felzenszwalb e IFT.
 * Opera em uma imagem fornecida no construtor.
 */
class ImageSegmenter {
public:
    /**
     * @brief Construtor da classe ImageSegmenter.
     * @param img A imagem de referência a ser segmentada (passada por referência constante).
     */
    ImageSegmenter(const Image& img);

    /**
     * @brief Realiza a segmentação da imagem usando o algoritmo de Felzenszwalb.
     * @param k Parâmetro de limiar para o Felzenszwalb. Controla a granularidade da segmentação.
     * @return Um vetor de inteiros, onde cada elemento é o rótulo do segmento correspondente ao pixel.
     */
    std::vector<int> segmentGraphFelzenszwalb(double k);

    /**
     * @brief Realiza a segmentação da imagem usando o algoritmo Image Forest Transform (IFT).
     * @param seeds Um vetor de índices de pixels que servem como sementes para o IFT.
     * @return Um vetor de inteiros, onde cada elemento é o rótulo da semente que dominou o pixel.
     */
    std::vector<int> segmentGraphIFT(const std::vector<int>& seeds);

    /**
     * @brief Visualiza a segmentação atribuindo cores aleatórias a cada rótulo único.
     * Retorna uma nova imagem colorida onde cada segmento é pintado com uma cor distinta.
     * @param labels Um vetor de rótulos de segmentação (saída de segmentGraphFelzenszwalb ou segmentGraphIFT).
     * @return Um objeto Image representando a visualização da segmentação.
     */
    Image visualizeSegmentation(const std::vector<int>& labels);

private:
    const Image& image; // Referência constante à imagem original
    int width, height;   // Dimensões da imagem para acesso rápido

    /**
     * @brief Constrói um vetor de todas as arestas possíveis entre pixels vizinhos (4-conectividade).
     * O peso de cada aresta é calculado pela função pixelDifference.
     * @return Um vetor de objetos Edge.
     */
    std::vector<Edge> buildEdges();

    /**
     * @brief Calcula a diferença/distância entre dois pixels.
     * Esta função é usada principalmente pelo algoritmo Felzenszwalb.
     * Utiliza a distância Euclidiana no espaço de cores RGB.
     * @param a O primeiro pixel.
     * @param b O segundo pixel.
     * @return A diferença calculada (peso da aresta).
     */
    double pixelDifference(const Pixel& a, const Pixel& b);

    /**
     * @brief Calcula a diferença/distância entre dois pixels especificamente para o IFT.
     * Esta versão pode aplicar um fator de amplificação para tornar as bordas mais "pesadas"
     * para o algoritmo IFT.
     * @param a O primeiro pixel.
     * @param b O segundo pixel.
     * @return A diferença calculada (peso da aresta amplificado para o IFT).
     */
    double pixelDifferenceIFT(const Pixel& a, const Pixel& b);

    /**
     * @brief Converte um pixel RGB para sua intensidade de escala de cinza (luminância).
     * @param p O pixel RGB.
     * @return O valor da intensidade de cinza (double).
     */
    double getGrayscaleIntensity(const Pixel& p) const;

    /**
     * @brief Constrói uma lista de adjacência para o grafo de pixels.
     * Cada entrada no vetor representa um pixel, e o vetor interno contém os índices de seus vizinhos.
     * (Usado principalmente pelo IFT para iteração sobre vizinhos).
     * @return Um vetor de vetores de inteiros representando a lista de adjacência.
     */
    std::vector<std::vector<int>> buildAdjacency();
};

#endif // IMAGE_SEGMENTER_H