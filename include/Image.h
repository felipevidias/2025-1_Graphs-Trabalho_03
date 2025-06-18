#ifndef IMAGE_H
#define IMAGE_H

#include <string>
#include <vector>
#include <cstdint> // Good practice to include here as well if Pixel might use it

struct Pixel {
    unsigned char r, g, b;
};

class Image {
public:
    int width, height;
    std::vector<Pixel> data;

    // Constructor to load from file (already exists)
    Image(const std::string& filename);

    // --- NEW CONSTRUCTOR HERE ---
    /**
     * @brief Construtor da classe Image para criar uma imagem vazia de um determinado tamanho.
     * @param w A largura da nova imagem.
     * @param h A altura da nova imagem.
     */
    Image(int w, int h) : width(w), height(h) {
        if (w > 0 && h > 0) {
            data.resize(w * h); // Aloca espaço para os pixels
        } else {
            width = 0; // Garante que a imagem é considerada vazia se w ou h for <= 0
            height = 0;
            data.clear();
        }
    }


    Pixel getPixel(int row, int col) const;
    void setPixel(int row, int col, const Pixel& p);
    void save(const std::string& filename) const;
    int index(int row, int col) const;
};

#endif // IMAGE_H