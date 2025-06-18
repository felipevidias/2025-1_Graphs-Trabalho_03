#include "ImageSegmenter.h" // Inclui o cabeçalho da própria classe
#include <cmath>            // Para funções matemáticas como sqrt, pow, abs
#include <algorithm>        // Para funções como std::sort, std::max
#include <unordered_map>    // Para std::unordered_map (usado em visualizeSegmentation)
#include <random>           // Para geração de números aleatórios (para cores em visualizeSegmentation)
#include <iostream>         // Para std::cout, std::cerr
#include <numeric>  // Para std::inner_product e outros

// Construtor da classe ImageSegmenter. Inicializa com a imagem fornecida.
ImageSegmenter::ImageSegmenter(const Image& img) : image(img), width(img.width), height(img.height) {}

// Implementação de getGrayscaleIntensity (usada por pixelDifference e pixelDifferenceIFT)
double ImageSegmenter::getGrayscaleIntensity(const Pixel& p) const {
    // Fórmula padrão de luminância para converter RGB para escala de cinza.
    return 0.299 * p.r + 0.587 * p.g + 0.114 * p.b;
}

// Implementação de pixelDifference (usada pelo Felzenszwalb)
// Calcula a distância Euclidiana no espaço de cores RGB.
// É eficaz para imagens coloridas e também funciona para tons de cinza.
double ImageSegmenter::pixelDifference(const Pixel& a, const Pixel& b) {
    double dr = a.r - b.r; // Diferença no canal Vermelho
    double dg = a.g - b.g; // Diferença no canal Verde
    double db = a.b - b.b; // Diferença no canal Azul
    double color_diff = sqrt(dr*dr + dg*dg + db*db); // Distância Euclidiana 3D
    return color_diff;
}

// Implementação de pixelDifferenceIFT (usada pelo IFT)
// Esta versão amplifica a distância de cor para tornar as bordas mais "visíveis" para o IFT.
double ImageSegmenter::pixelDifferenceIFT(const Pixel& a, const Pixel& b) {
    double dr = a.r - b.r;
    double dg = a.g - b.g;
    double db = a.b - b.b;
    
    double color_distance = sqrt(dr*dr + dg*dg + db*db); // Distância Euclidiana RGB

    // Fator de amplificação para os pesos das arestas no IFT.
    // Ajuste este valor:
    // - Um valor maior fará com que o IFT seja mais sensível às diferenças de cor,
    //   tendendo a criar mais segmentos e respeitar bordas mais sutis.
    // - Um valor menor fará com que o IFT se expanda mais livremente,
    //   resultando em menos segmentos (blocos maiores).
    // Para imagens como 'lego.png' com bordas fortes, um fator alto é benéfico.
    const double AMPLIFICATION_FACTOR = 250.0; 

    return color_distance * AMPLIFICATION_FACTOR; 
}

// Constrói um vetor de todas as arestas entre pixels vizinhos (4-conectividade).
std::vector<Edge> ImageSegmenter::buildEdges(const Image& sourceImage) {
    std::vector<Edge> edges;
    for (int r = 0; r < height; ++r) {
        for (int c = 0; c < width; ++c) {
            int u = sourceImage.index(r, c);
            if (c + 1 < width) {
                int v = sourceImage.index(r, c + 1);
                // Calcula a diferença usando a imagem fornecida (sourceImage)
                edges.push_back({u, v, pixelDifference(sourceImage.getPixel(r, c), sourceImage.getPixel(r, c + 1))});
            }
            if (r + 1 < height) {
                int v = sourceImage.index(r + 1, c);
                // Calcula a diferença usando a imagem fornecida (sourceImage)
                edges.push_back({u, v, pixelDifference(sourceImage.getPixel(r, c), sourceImage.getPixel(r + 1, c))});
            }
        }
    }
    return edges;
}

/**
 * @brief Cria uma nova imagem suavizada com um filtro Gaussiano.
 * @param sigma O desvio padrão do filtro. Valores maiores produzem mais desfoque.
 * @return Um novo objeto Image contendo os dados suavizados.
 */
Image ImageSegmenter::createSmoothedImage(double sigma) {
    Image smoothedImage = image; // Cria uma cópia da imagem original para modificar

    int kernelSize = static_cast<int>(6 * sigma) | 1;
    int kernelRadius = kernelSize / 2;
    std::vector<double> kernel(kernelSize * kernelSize);
    double sum = 0.0;

    // Cria o kernel Gaussiano 2D
    for (int j = -kernelRadius; j <= kernelRadius; ++j) {
        for (int i = -kernelRadius; i <= kernelRadius; ++i) {
            double r = std::sqrt(i * i + j * j);
            kernel[(j + kernelRadius) * kernelSize + (i + kernelRadius)] = (std::exp(-(r * r) / (2 * sigma * sigma)) / (2 * M_PI * sigma * sigma));
            sum += kernel[(j + kernelRadius) * kernelSize + (i + kernelRadius)];
        }
    }
    for (int i = 0; i < kernelSize * kernelSize; ++i) kernel[i] /= sum;

    // Aplica a convolução do kernel na imagem original
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            double r_sum = 0.0, g_sum = 0.0, b_sum = 0.0;
            for (int j = -kernelRadius; j <= kernelRadius; ++j) {
                for (int i = -kernelRadius; i <= kernelRadius; ++i) {
                    int pixelX = std::min(width - 1, std::max(0, x + i));
                    int pixelY = std::min(height - 1, std::max(0, y + j));
                    
                    const Pixel& p = image.data[pixelY * width + pixelX];
                    double kernelValue = kernel[(j + kernelRadius) * kernelSize + (i + kernelRadius)];
                    r_sum += p.r * kernelValue;
                    g_sum += p.g * kernelValue;
                    b_sum += p.b * kernelValue;
                }
            }
            smoothedImage.data[y * width + x] = {
                static_cast<uint8_t>(r_sum),
                static_cast<uint8_t>(g_sum),
                static_cast<uint8_t>(b_sum)
            };
        }
    }
    return smoothedImage;
}

// Implementação do algoritmo de segmentação Felzenszwalb.
std::vector<int> ImageSegmenter::segmentGraphFelzenszwalb(double k, double sigma) {
    int n = width * height;

    // 1. Cria uma versão suavizada da imagem antes de qualquer outra coisa.
    // O valor de sigma controla a intensidade do desfoque. O artigo sugere 0.8.
    Image smoothedImage = createSmoothedImage(sigma);
    
    // 2. Constrói as arestas do grafo USANDO A IMAGEM SUAVIZADA.
    std::vector<Edge> edges = buildEdges(smoothedImage); 
    
    // 3. O resto do algoritmo permanece o mesmo.
    DSU dsu(n);
    std::sort(edges.begin(), edges.end(), [](Edge a, Edge b) { return a.weight < b.weight; });

    for (const Edge& e : edges) {
        dsu.unite(e.u, e.v, e.weight, k);
    }

    std::vector<int> labels(n);
    for (int i = 0; i < n; ++i) labels[i] = dsu.find(i);
    return labels;
}

// Constrói uma lista de adjacência para o grafo de pixels (4-conectividade).
std::vector<std::vector<int>> ImageSegmenter::buildAdjacency() {
    std::vector<std::vector<int>> adj(width * height); // Lista de adjacência, um vetor para cada pixel.
    // Itera sobre todos os pixels da imagem.
    for (int r = 0; r < height; ++r) {
        for (int c = 0; c < width; ++c) {
            int u = image.index(r, c); // Índice do pixel atual

            // Adiciona vizinhos para 4-conectividade (cima, baixo, esquerda, direita).
            if (r + 1 < height) adj[u].push_back(image.index(r + 1, c)); // Vizinho abaixo
            if (c + 1 < width) adj[u].push_back(image.index(r, c + 1));   // Vizinho à direita
            if (r - 1 >= 0) adj[u].push_back(image.index(r - 1, c));     // Vizinho acima
            if (c - 1 >= 0) adj[u].push_back(image.index(r, c - 1));     // Vizinho à esquerda
        }
    }
    return adj; // Retorna a lista de adjacência.
}

/**
 * @brief Calcula a magnitude do gradiente da imagem usando o operador Sobel.
 *
 * Esta versão foi CORRIGIDA para trabalhar com um `std::vector<Pixel>`,
 * acessando os membros .r, .g, .b de cada pixel para a conversão para tons de cinza.
 *
 * @return Um vetor de doubles contendo o mapa de gradiente normalizado.
 */
std::vector<double> ImageSegmenter::calculateGradientMagnitude() {
    int width = image.width;
    int height = image.height;
    int n = width * height;

    if (n == 0) {
        std::cerr << "AVISO: Tentando calcular o gradiente de uma imagem vazia." << std::endl;
        return {};
    }

    std::vector<double> grayscaleImage(n);
    
    // --- CORREÇÃO PRINCIPAL ---
    // 1. Converter imagem para tons de cinza
    // Iteramos sobre o vetor de Pixels. Para cada Pixel, calculamos seu
    // valor de intensidade (tons de cinza) usando seus membros .r, .g e .b.
    for (int i = 0; i < n; ++i) {
        const Pixel& p = image.data[i];
        // Fórmula de luminosidade padrão para conversão RGB -> Grayscale
        // Se sua struct Pixel usa nomes diferentes (ex: red, green, blue), ajuste aqui.
        grayscaleImage[i] = 0.299 * p.r + 0.587 * p.g + 0.114 * p.b;
    }
    
    // O restante da função, que opera sobre a imagem em tons de cinza,
    // permanece exatamente o mesmo, pois já está correto.

    // 2. Definir os kernels (máscaras) do operador Sobel
    const double Gx[3][3] = {{-1, 0, 1},
                             {-2, 0, 2},
                             {-1, 0, 1}};

    const double Gy[3][3] = {{-1, -2, -1},
                             { 0,  0,  0},
                             { 1,  2,  1}};

    std::vector<double> gradientImage(n, 0.0);
    double max_gradient = 0.0;

    // 3. Aplicar os kernels na imagem em tons de cinza
    for (int y = 1; y < height - 1; ++y) {
        for (int x = 1; x < width - 1; ++x) {
            double sum_gx = 0.0;
            double sum_gy = 0.0;

            for (int j = -1; j <= 1; ++j) {
                for (int i = -1; i <= 1; ++i) {
                    double pixel_val = grayscaleImage[(y + j) * width + (x + i)];
                    sum_gx += pixel_val * Gx[j + 1][i + 1];
                    sum_gy += pixel_val * Gy[j + 1][i + 1];
                }
            }
            
            double magnitude = std::sqrt(sum_gx * sum_gx + sum_gy * sum_gy);
            int currentIndex = y * width + x;
            gradientImage[currentIndex] = magnitude;

            if (magnitude > max_gradient) {
                max_gradient = magnitude;
            }
        }
    }

    // 4. Normalizar a imagem de gradiente para o intervalo [0, 255]
    if (max_gradient > 0) {
        for (int i = 0; i < n; ++i) {
            gradientImage[i] = (gradientImage[i] / max_gradient) * 255.0;
        }
    }

    return gradientImage;
}

// Implementação do algoritmo Image Forest Transform (IFT).
std::vector<int> ImageSegmenter::segmentGraphIFT(const std::vector<double>& gradientImage, const std::vector<int>& seeds) 
{
    int n = width * height; // Número total de pixels.
    std::vector<double> cost(n, 1e9); // Custo do caminho da semente até o pixel (inicialmente infinito).
    std::vector<int> label(n, -1);    // Rótulo do pixel (inicialmente -1, sem rótulo).
    // Fila de prioridade para o algoritmo de Dijkstra (IFT é um tipo de Dijkstra).
    // Armazena pares {custo, índice_pixel}, ordenados pelo menor custo.
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> pq;

    // Inicializa as sementes na fila de prioridade.
    for (int s : seeds) {
        cost[s] = 0;   // O custo da semente para si mesma é 0.
        label[s] = s;  // O rótulo da semente é seu próprio índice.
        pq.push({0, s}); // Adiciona a semente na fila de prioridade.
    }

    auto adj = buildAdjacency(); // Constrói a lista de adjacência para iterar sobre vizinhos.

    // Variáveis de depuração para controlar a impressão no console.
    int debug_counter = 0;
    const int DEBUG_PRINT_INTERVAL = 10000; // Imprime a cada 10.000 iterações.

    // Loop principal do IFT: Enquanto houver pixels na fila de prioridade.
    while (!pq.empty()) { 
        auto [c, u] = pq.top(); // Obtém o pixel 'u' com o menor custo 'c' da fila.
        pq.pop();               // Remove o pixel da fila.

        // Se o custo atual 'c' é maior do que o custo já conhecido para 'u',
        // significa que já encontramos um caminho mais curto, então ignoramos.
        if (c > cost[u]) continue;

        // Itera sobre todos os vizinhos 'v' do pixel atual 'u'.
        for (int v : adj[u]) {
            // Calcula o peso da aresta entre 'u' e 'v' usando pixelDifferenceIFT (amplificado).
            double w = gradientImage[v];
            // Calcula o novo custo para 'v' através de 'u': custo acumulado (cost[u] + w).
            double new_cost = std::max(cost[u], w);  // fmax(π⋅⟨s,t⟩) = max{fmax(π),w(s,t)}

            // --- LINHAS DE DEPURAGEM (COMENTADAS POR PADRÃO) ---
            // Descomente para ver o fluxo detalhado do IFT no console.
            /*
            debug_counter++;
            if (debug_counter % DEBUG_PRINT_INTERVAL == 0) {
                std::cout << "[DEBUG IFT] u=" << u << ", v=" << v << ", w=" << w
                          << ", cost[u]=" << cost[u] << ", new_cost=" << new_cost
                          << ", cost[v]=" << cost[v] << std::endl;
            }
            */
            // --- FIM DAS LINHAS DE DEPURAGEM ---
            
            // Se o novo custo para 'v' é menor do que o custo atualmente conhecido para 'v'.
            if (new_cost < cost[v]) {
                cost[v] = new_cost;   // Atualiza o custo de 'v'.
                label[v] = label[u];  // Atribui o rótulo de 'u' a 'v' (propaga o rótulo da semente).
                pq.push({new_cost, v}); // Adiciona 'v' na fila de prioridade com o novo custo.
            }
        }
    } 

    return label; // Retorna o vetor de rótulos de segmentação.
} 

// Implementação da visualização da segmentação.
Image ImageSegmenter::visualizeSegmentation(const std::vector<int>& labels) {
    Image out = image; // Cria uma cópia da imagem original para a saída.
    std::unordered_map<int, Pixel> color_map; // Mapeia cada rótulo único para uma cor aleatória.
    std::mt19937 rng(42); // Gerador de números pseudoaleatórios com semente fixa para reprodutibilidade.
    std::uniform_int_distribution<int> dist(0, 255); // Distribuição para gerar valores RGB de 0 a 255.

    bool first_color_generated = false; // Flag de depuração para a primeira cor gerada.

    // Itera sobre todos os pixels (e seus rótulos).
    for (size_t i = 0; i < labels.size(); ++i) {
        int lbl = labels[i]; // Obtém o rótulo do pixel atual.
        // Se o rótulo ainda não tem uma cor associada, gera uma nova cor aleatória.
        if (color_map.find(lbl) == color_map.end()) {
            color_map[lbl] = { (unsigned char)dist(rng), (unsigned char)dist(rng), (unsigned char)dist(rng) };
            // Imprime a primeira cor gerada para depuração.
            if (!first_color_generated) {
                std::cout << "[DEBUG - Visualize] Primeira cor gerada para rótulo " << lbl << ": R=" << (int)color_map[lbl].r
                          << ", G=" << (int)color_map[lbl].g << ", B=" << (int)color_map[lbl].b << std::endl;
                first_color_generated = true;
            }
        }
        // Atribui a cor mapeada (ou recém-gerada) ao pixel na imagem de saída.
        out.data[i] = color_map[lbl];
    }

    return out; // Retorna a imagem segmentada visualizada.
}