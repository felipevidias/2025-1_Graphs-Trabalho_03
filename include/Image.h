#ifndef IMAGE_H // Previne inclusões múltiplas do mesmo cabeçalho
#define IMAGE_H

#include <string> // Necessário para usar std::string (nomes de arquivos)
#include <vector> // Necessário para usar std::vector (dados de pixels)

/**
 * @brief Estrutura que representa um pixel em formato RGB.
 * Cada componente de cor (vermelho, verde, azul) é um unsigned char (0-255).
 */
struct Pixel {
    unsigned char r, g, b; // Componentes de cor: Red, Green, Blue
};

/**
 * @brief Classe para manipulação de imagens (carregamento, acesso a pixels, salvamento).
 * Suporta imagens em formato RGB.
 */
class Image {
public:
    int width, height; // Largura e altura da imagem em pixels
    std::vector<Pixel> data; // Vetor que armazena os dados dos pixels da imagem
                               // Os pixels são armazenados em ordem linear (linha a linha).

    /**
     * @brief Construtor da classe Image. Carrega uma imagem de um arquivo.
     * @param filename O caminho para o arquivo da imagem (ex: "path/to/image.png").
     */
    Image(const std::string& filename);

    /**
     * @brief Obtém o pixel em uma determinada linha e coluna.
     * @param row A linha do pixel (0 a height-1).
     * @param col A coluna do pixel (0 a width-1).
     * @return O objeto Pixel na posição especificada.
     */
    Pixel getPixel(int row, int col) const;

    /**
     * @brief Define o pixel em uma determinada linha e coluna com um novo valor.
     * @param row A linha do pixel.
     * @param col A coluna do pixel.
     * @param p O novo valor do pixel.
     */
    void setPixel(int row, int col, const Pixel& p);

    /**
     * @brief Salva a imagem atual em um arquivo PNG.
     * @param filename O caminho para o arquivo onde a imagem será salva.
     */
    void save(const std::string& filename) const;

    /**
     * @brief Converte coordenadas de linha/coluna para um índice linear no vetor 'data'.
     * @param row A linha do pixel.
     * @param col A coluna do pixel.
     * @return O índice linear correspondente no vetor 'data'.
     */
    int index(int row, int col) const;
};

#endif // IMAGE_H