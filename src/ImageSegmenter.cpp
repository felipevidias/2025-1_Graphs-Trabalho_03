#include "ImageSegmenter.h" // Inclui o cabeçalho da própria classe
#include <cmath>            // Para funções matemáticas como sqrt, pow, abs
#include <algorithm>        // Para funções como std::sort, std::max
#include <unordered_map>    // Para std::unordered_map (usado em visualizeSegmentation)
#include <random>           // Para geração de números aleatórios (para cores em visualizeSegmentation)
#include <iostream>         // Para std::cout, std::cerr

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
std::vector<Edge> ImageSegmenter::buildEdges() {
    std::vector<Edge> edges;
    // Itera sobre todos os pixels da imagem.
    for (int r = 0; r < height; ++r) {
        for (int c = 0; c < width; ++c) {
            int u = image.index(r, c); // Índice do pixel atual
            
            // Adiciona aresta para o vizinho à direita, se existir.
            if (c + 1 < width) {
                int v = image.index(r, c + 1);
                // O peso da aresta é a diferença entre os pixels, calculada por pixelDifference (para Felzenszwalb).
                edges.push_back({u, v, pixelDifference(image.getPixel(r, c), image.getPixel(r, c + 1))});
            }
            // Adiciona aresta para o vizinho abaixo, se existir.
            if (r + 1 < height) {
                int v = image.index(r + 1, c);
                // O peso da aresta é a diferença entre os pixels, calculada por pixelDifference (para Felzenszwalb).
                edges.push_back({u, v, pixelDifference(image.getPixel(r, c), image.getPixel(r + 1, c))});
            }
        }
    }
    return edges;
}

// Implementação do algoritmo de segmentação Felzenszwalb.
std::vector<int> ImageSegmenter::segmentGraphFelzenszwalb(double k) {
    int n = width * height; // Número total de pixels na imagem.

    // Mensagens de depuração para as dimensões da imagem.
    std::cout << "[DEBUG] Felzenszwalb - Largura: " << width << ", Altura: " << height << ", Total Pixels (n): " << n << std::endl;
    if (n <= 0) {
        std::cerr << "[ERRO] Felzenszwalb - Dimensões da imagem inválidas. Largura e Altura devem ser positivos!" << std::endl;
        exit(1); // Sai do programa em caso de erro crítico.
    }

    DSU dsu(n); // Cria uma instância da estrutura Disjoint Set Union (DSU).
    std::vector<Edge> edges = buildEdges(); // Constrói todas as arestas do grafo de pixels.
    // Ordena as arestas pelo peso (diferença entre pixels) em ordem crescente.
    std::sort(edges.begin(), edges.end(), [](Edge a, Edge b) { return a.weight < b.weight; });

    // Itera sobre as arestas ordenadas e tenta uni-las.
    for (const Edge& e : edges) {
        // Tenta unir os dois vértices (pixels) da aresta.
        // A união ocorre se a diferença entre eles (e.weight) não for muito maior
        // que a "variação interna" dos componentes já existentes (controlado por k).
        dsu.unite(e.u, e.v, e.weight, k);
    }

    std::vector<int> labels(n); // Vetor para armazenar os rótulos de segmentação.
    // Para cada pixel, encontra o representante (raiz) do seu conjunto na DSU.
    // Este representante serve como o rótulo do segmento.
    for (int i = 0; i < n; ++i) labels[i] = dsu.find(i);
    return labels; // Retorna os rótulos de segmentação.
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

// Implementação do algoritmo Image Forest Transform (IFT).
std::vector<int> ImageSegmenter::segmentGraphIFT(const std::vector<int>& seeds) {
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
            double w = pixelDifferenceIFT(image.data[u], image.data[v]);
            // Calcula o novo custo para 'v' através de 'u': custo acumulado (cost[u] + w).
            double new_cost = cost[u] + w;

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