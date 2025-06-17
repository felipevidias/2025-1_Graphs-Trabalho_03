// Esta macro define a implementação da biblioteca stb_image para carregamento de imagens.
// Deve ser definida APENAS uma vez em um arquivo .cpp do projeto.
#define STB_IMAGE_IMPLEMENTATION
// Esta macro define a implementação da biblioteca stb_image_write para salvamento de imagens.
// Deve ser definida APENAS uma vez em um arquivo .cpp do projeto.
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include <iostream>      // Para std::cerr (saída de erro)
#include "Image.h"       // Inclui a definição da classe Image
#include "stb_image.h"   // Biblioteca para carregar imagens
#include "stb_image_write.h" // Biblioteca para salvar imagens

/**
 * @brief Construtor da classe Image. Carrega uma imagem de um arquivo.
 * Utiliza a biblioteca stb_image para carregar imagens em vários formatos.
 * @param filename O caminho para o arquivo da imagem.
 */
Image::Image(const std::string& filename) {
    int channels; // Variável para armazenar o número de canais da imagem carregada
    unsigned char* img = stbi_load(filename.c_str(), &width, &height, &channels, 3); // Carrega a imagem
    // O '3' no final força o carregamento como 3 canais (RGB), mesmo que seja tons de cinza ou tenha alfa.

    // Verifica se o carregamento da imagem falhou.
    if (!img) {
        std::cerr << "[ERRO] Falha ao carregar a imagem: " << filename << "\n";
        std::cerr << "[ERRO] Detalhes: " << stbi_failure_reason() << "\n"; // Mensagem de erro da stb_image
        width = height = 0; // Define dimensões como zero para indicar falha
        return;
    }

    std::cout << "[DEBUG] Imagem '" << filename << "' carregada. Largura: " << width
              << ", Altura: " << height << ", Canais originais: " << channels << "\n";

    // Redimensiona o vetor de dados para armazenar todos os pixels da imagem (width * height).
    data.resize(width * height);
    // Copia os dados da imagem carregada para o vetor 'data' da classe.
    // stbi_load retorna os pixels em ordem R, G, B, R, G, B...
    for (int i = 0; i < width * height; ++i) {
        data[i] = { img[3 * i], img[3 * i + 1], img[3 * i + 2] };
    }

    stbi_image_free(img); // Libera a memória alocada pela stb_image.
}

/**
 * @brief Obtém o valor do pixel em uma determinada linha e coluna.
 * @param r A linha do pixel (row).
 * @param c A coluna do pixel (column).
 * @return O Pixel na posição especificada.
 */
Pixel Image::getPixel(int r, int c) const {
    // Calcula o índice linear a partir da linha e coluna.
    return data[r * width + c];
}

/**
 * @brief Define o valor de um pixel em uma determinada linha e coluna.
 * @param r A linha do pixel.
 * @param c A coluna do pixel.
 * @param p O novo valor Pixel.
 */
void Image::setPixel(int r, int c, const Pixel& p) {
    // Calcula o índice linear e define o pixel.
    data[r * width + c] = p;
}

/**
 * @brief Salva a imagem atual em um arquivo PNG.
 * Utiliza a biblioteca stb_image_write para salvar a imagem.
 * @param filename O caminho para o arquivo onde a imagem será salva (com extensão .png).
 */
void Image::save(const std::string& filename) const {
    // stbi_write_png(caminho, largura, altura, num_canais, dados_pixels, stride)
    // stride = largura * num_canais (para dados compactos)
    stbi_write_png(filename.c_str(), width, height, 3, data.data(), width * 3);
}

/**
 * @brief Converte coordenadas de linha/coluna para um índice linear.
 * @param r A linha do pixel.
 * @param c A coluna do pixel.
 * @return O índice linear no vetor de pixels.
 */
int Image::index(int r, int c) const {
    return r * width + c;
}