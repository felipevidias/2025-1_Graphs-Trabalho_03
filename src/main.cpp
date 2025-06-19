#include "Image.h"          // Inclui a definição da classe Image para manipulação de imagens
#include "ImageSegmenter.h" // Inclui a definição da classe ImageSegmenter
#include <iostream>         // Para entrada e saída de dados (cout, cerr)
#include <string>           // Para manipulação de strings (nomes de arquivos)
#include <set>              // Usado para contar rótulos únicos na depuração do IFT
#include <cstdint>          // Necessário para uint8_t
#include <chrono>          // Para manipulação de tempo (opcional, mas útil para medir desempenho)

/**
 * @brief Extrai o nome base de um caminho de arquivo, removendo diretórios e extensão.
 * @param path O caminho completo do arquivo (ex: "src/minha_imagem.png").
 * @return O nome base do arquivo (ex: "minha_imagem").
 */
std::string getBaseName(const std::string& path) {
    // Encontra a última ocorrência de '/' ou '\' para o separador de diretório
    size_t slash = path.find_last_of("/\\");
    // Encontra a última ocorrência de '.' para a extensão do arquivo
    size_t dot = path.find_last_of(".");
    
    // Se não houver ponto (sem extensão), considera que o nome vai até o final da string
    if (dot == std::string::npos) dot = path.length();
    
    // Retorna a substring que corresponde ao nome base
    // Começa após o último separador de diretório (ou do início, se não houver)
    // e termina antes do ponto da extensão (ou do final da string)
    return path.substr(slash + 1, dot - slash - 1);
}

/**
 * @brief Função principal do programa.
 * Lida com o carregamento da imagem, execução dos algoritmos de segmentação
 * (Felzenszwalb e IFT) e salvamento dos resultados.
 * @return 0 se a execução for bem-sucedida, 1 em caso de erro.
 */
int main() {
    // *** DEFINIÇÃO DA IMAGEM DE TESTE ***
    // Altere este caminho para a imagem que deseja testar.
    // Certifique-se de que a imagem (ex: lego.png, coffe-table.png, lenna-RGB.png, lennaGray.png)
    // está localizada no diretório 'src/'.
    std::string inputFilename = "src/lennaGray.png"; // Imagem de entrada atual

    // Cria um objeto Image carregando a imagem do caminho especificado.
    Image img(inputFilename);

    // Verificação básica para garantir que a imagem foi carregada corretamente.
    if (img.width == 0 || img.height == 0) {
        std::cerr << "ERRO: Não foi possível carregar a imagem: " << inputFilename << ". Verifique o caminho ou se ela existe." << std::endl;
        return 1; // Sai com código de erro se a imagem não puder ser carregada.
    }

    // Cria um objeto ImageSegmenter associado à imagem carregada.
    ImageSegmenter segmenter(img);

    // Obtém o nome base do arquivo (sem o caminho ou extensão) para nomear os arquivos de saída.
    std::string baseName = getBaseName(inputFilename);

    // --- EXECUÇÃO E SALVAMENTO DA SEGMENTAÇÃO FELZENSZWALB ---
    // O parâmetro 'k' controla a granularidade da segmentação Felzenszwalb.
    // - Valores maiores de 'k' resultam em menos segmentos (maiores e mais homogêneos).
    // - Valores menores de 'k' resultam em mais segmentos (menores e mais detalhados).
    // O valor ideal de 'k' é experimental e depende da imagem de entrada.
    // O artigo usa sigma = 0.8 
    // O valor de 'k' geralmente precisa ser aumentado para imagens suavizadas.
    // Experimente valores de k entre 300 e 1000.

    // Inicia o cronômetro para o Felzenszwalb
    auto start_fz = std::chrono::high_resolution_clock::now();
    auto labels_fz = segmenter.segmentGraphFelzenszwalb(6000.0, 0.8);   
    // Para o cronômetro para o Felzenszwalb
    auto end_fz = std::chrono::high_resolution_clock::now();
    // Calcula a duração em milissegundos
    auto duration_fz = std::chrono::duration_cast<std::chrono::milliseconds>(end_fz - start_fz); 

    auto out_fz = segmenter.visualizeSegmentation(labels_fz); // Visualiza os rótulos em cores aleatórias
    out_fz.save("outputs/" + baseName + "_felzenszwalb.png");  // Salva a imagem segmentada
    std::cout << "Segmentação Felzenszwalb gerada: " << baseName << "_felzenszwalb.png" << std::endl;

    // --- RECORTANDO UM SEGMENTO ESPECÍFICO COM FELZENSZWALB ---
    // Escolha um rótulo de segmento para recortar.
    // Você precisará inspecionar a imagem '_felzenszwalb.png' para encontrar um rótulo interessante.
    // Por exemplo, para 'coffe-table.png', o rótulo do pixel (310, 260) pode ser parte da xícara da frente.
    int target_segment_label_fz = labels_fz[img.index(310, 260)]; // Exemplo: rótulo do pixel central da xícara da frente
    Pixel background_color_fz = {0, 0, 0}; // Cor de fundo preta para áreas fora do segmento recortado
    Image cropped_fz_segment = segmenter.cropSegment(labels_fz, target_segment_label_fz, background_color_fz);
    if (cropped_fz_segment.width > 0 && cropped_fz_segment.height > 0) {
        cropped_fz_segment.save("outputs/" + baseName + "_felzenszwalb_cropped_segment.png");
        std::cout << "Segmento Felzenszwalb recortado (" << target_segment_label_fz << ") gerado: " 
                  << baseName << "_felzenszwalb_cropped_segment.png" << std::endl;
    } else {
        std::cout << "Falha ao recortar segmento Felzenszwalb ou segmento não encontrado." << std::endl;
    }


    // --- EXECUÇÃO E SALVAMENTO DA SEGMENTAÇÃO IFT ---

    // 1. CALCULAR IMAGEM DE GRADIENTE (Pré-processamento necessário para o Watershed)
    // Conforme justificado pelo artigo, o IFT-Watershed opera sobre as "cristas"
    // de uma imagem de gradiente, e não na imagem de cores original.

    // Inicia o cronômetro para o IFT (incluindo cálculo de gradiente)
    auto start_ift = std::chrono::high_resolution_clock::now();

    std::cout << "\nCalculando imagem de gradiente para o IFT-Watershed..." << std::endl;
    auto gradientImageVector = segmenter.calculateGradientMagnitude();

    // --- SALVAR A IMAGEM DE GRADIENTE PARA VISUALIZAÇÃO ---
    Image gradientVisImage = img; // Cria uma cópia da imagem original para usar como base
    for (int i = 0; i < gradientVisImage.width * gradientVisImage.height; ++i) {
        // Converte o valor double do gradiente para um valor de 8-bit (0-255)
        uint8_t gray_val = static_cast<uint8_t>(gradientImageVector[i]);
        // Define o pixel como um tom de cinza (R=G=B)
        // CORREÇÃO AQUI: Crie um objeto Pixel temporário e atribua-o
        gradientVisImage.data[i] = Pixel{gray_val, gray_val, gray_val};
    }

    gradientVisImage.save("outputs/" + baseName + "_gradient.png");
    std::cout << "Imagem de gradiente gerada: " << baseName << "_gradient.png" << std::endl;
    
    // As 'seeds' (sementes) são pontos iniciais a partir dos quais o IFT expande as regiões.
    // Seu posicionamento é crucial para o resultado do IFT.
    // As coordenadas (linha, coluna) são convertidas para um índice linear usando img.index().
    // Estas sementes são escolhidas especificamente para a imagem 'lego.png' para cobrir
    // diferentes objetos (caixa, blocos coloridos, fundo).
    // std::vector<int> seeds = {
    //     img.index(100, 250),   // Topo da caixa (exemplo)
    //     img.index(300, 400),   // Lado da caixa (exemplo)
    //     img.index(400, 100),   // Bloco azul (exemplo)
    //     img.index(400, 200),   // Bloco vermelho (exemplo)
    //     img.index(400, 300),   // Bloco amarelo (exemplo)
    //     img.index(50, 450),    // Fundo branco (exemplo)
    //     img.index(450, 450)    // Fundo branco (exemplo)
    // };

    // --- REFERÊNCIAS DE SEMENTES PARA OUTRAS IMAGENS (COMENTADAS) ---
    // Mantenha estas linhas comentadas; elas servem apenas como exemplo e referência
    // para outras imagens que você possa ter testado ou queira testar no futuro.
    // SEEDS PARA A IMAGEM 'coffe-table.png':
    // std::vector<int> seeds = {
    //     // --- Xícara da Frente (Marrom) ---
    //     img.index(310, 260),   // Interior da xícara (café)
    //     img.index(350, 250),   // Pires da xícara

    //     // --- Xícara de Trás (Verde) ---
    //     img.index(155, 350),   // Interior da xícara (espuma)
    //     img.index(210, 350),   // Pires da xícara

    //     // --- Livro e Óculos ---
    //     img.index(225, 100),   // Capa do livro (inferior)
    //     img.index(133, 105),   // Capa do livro (superior)
    //     img.index(138, 189),   // Capa do livro (superior-direita)
    //     img.index(218, 180),    // Borda lateral do livro (páginas)
    //     img.index(275, 60),    // Borda inferior do livro (páginas)
    //     img.index(150, 135),   // Óculos (lente direita)
    //     img.index(195, 65),    // Óculos (lente esquerda)

    //     // --- Mesa e Fundo ---
    //     img.index(140, 40),    // Mesa (próximo ao livro)
    //     img.index(320, 55),    // Mesa (meio-esquerda)
    //     img.index(450, 370),   // Mesa (canto inferior direito)
    //     img.index(460, 135),   // Mesa (canto inferior esquerdo)
    //     img.index(85, 230),    // Mesa (canto superior direito)
    //     img.index(85, 25),      // Fundo (canto superior esquerdo)
    //     img.index(30, 200),      // Fundo
    // };
    // SEEDS PARA A IMAGEM 'cooper.png':
    // std::vector<int> seeds = {
    //     // --- Cabelo ---
    //     img.index(90, 245),   

    //     // --- Rosto e Pele ---
    //     img.index(160, 221),   // Testa
    //     img.index(165, 168),   // Testa (sombra)
    //     img.index(262, 219),   // Nariz
    //     img.index(243, 209),   // Nariz (sombra)
    //     img.index(269, 293),   // Bochecha (à nossa direita)
    //     img.index(290, 178),   // Bochecha (à nossa esquerda)
    //     img.index(343, 226),   // Queixo    
    //     img.index(229, 345),   // Orelha

    //     // --- Olhos e Lábios (Detalhes Finos) ---
    //     img.index(210, 265),   // Olho (à nossa direita)
    //     img.index(217, 188),   // Olho (à nossa esquerda)
    //     img.index(300, 225),   // Lábios (superior)
    //     img.index(308, 227),   // Lábios (inferior)
    //     img.index(195, 267),   // Sobrancelha (à nossa direita)
    //     img.index(201, 189),   // Sobrancelha (à nossa esquerda)

    //     // --- Roupa ---
    //     img.index(423, 282),    // Camisa (direita)
    //     img.index(414, 205),    // Camisa (esquerda)
    //     img.index(464, 223),    // Gravata
    //     img.index(455, 380),   // Terno (direita)
    //     img.index(458, 141),   // Terno (esquerda)

    //     // --- Fundo ---
    //     img.index(32, 44),     // Fundo (canto superior esquerdo)
    //     img.index(441, 53),     // Fundo (canto inferior esquerdo)
    //     img.index(362, 450),    // Fundo (canto inferior direito)
    //     img.index(78, 450),    // Fundo (canto superior direito)
    //     img.index(28, 242)         // Fundo (cima)
    // };
    // SEEDS PARA A IMAGEM 'lenna-RGB.png':
    std::vector<int> seeds = {
        // --- Rosto e Pele ---
        img.index(225, 306),   // Testa
        img.index(310, 240),   // Bochecha (à nossa esquerda)
        img.index(300, 300),   // Nariz
        img.index(385, 275),   // Queixo
        img.index(440, 320),   // Ombro (à frente)
        img.index(465, 250),   // Ombro (atrás)
        img.index(470, 330),   // Ombro (baixo)

        // --- Olhos e Lábios (Detalhes Finos) ---
        img.index(261, 263),   // Olho (à nossa esquerda)
        img.index(262, 326),   // Olho (à nossa direita)
        img.index(347, 291),   // Lábios

        // --- Chapéu ---
        img.index(66, 161),    // Topo do chapéu
        img.index(91, 182),   // Meio do chapéu (área de textura)
        img.index(145, 225),   // Fita do chapéu
        img.index(185, 368),   // Aba do chapéu (próxima ao fundo)
        img.index(185, 308),   // Aba do chapéu (próxima ao cabelo)

        // --- Cabelo ---
        img.index(359, 344),   // Cabelo próximo ao rosto
        img.index(386, 186),   // Cabelo nas costas

        // --- Pluma Roxa (Textura Complexa) ---
        img.index(218, 201),   // Parte de cima da pluma
        img.index(321, 109),   // Meio da pluma
        img.index(362, 92),   // Meio da pluma
        img.index(462, 106),    // Ponta de baixo da pluma

        // --- Fundo ---
        img.index(230, 81),    // Fundo vermelho/rosa (à esquerda)
        img.index(19, 221),    // Fundo vermelho/rosa (topo)
        img.index(354, 474),   // Reflexo no espelho (objeto amarelo)
        img.index(204, 398),   // borda do espelho
        img.index(60, 475),    // borda do espelho (cima)
        img.index(105, 372),   // Área escura (canto superior direito)
        img.index(371, 5),    // Área escura (atrás das costas)
        img.index(420, 43)    // Pilar (atrás das costas)
    };
    

    // Verificação de segurança para garantir que todas as sementes estão dentro dos limites da imagem.
    for (size_t i = 0; i < seeds.size(); ++i) {
        if (seeds[i] < 0 || seeds[i] >= img.width * img.height) {
            std::cerr << "ERRO: Semente " << i << " fora dos limites da imagem: " << seeds[i] << std::endl;
            return 1; // Sai com código de erro se uma semente for inválida.
        }
    }

    // Executa o algoritmo IFT com as sementes definidas.
    auto labels_ift = segmenter.segmentGraphIFT(gradientImageVector, seeds);

    // Para o cronômetro para o IFT
    auto end_ift = std::chrono::high_resolution_clock::now();
    // Calcula a duração em milissegundos
    auto duration_ift = std::chrono::duration_cast<std::chrono::milliseconds>(end_ift - start_ift);

    auto out_ift = segmenter.visualizeSegmentation(labels_ift); // Visualiza os rótulos
    out_ift.save("outputs/" + baseName + "_ift_watershed.png"); // Salva a imagem segmentada
    std::cout << "Segmentação IFT-Watershed gerada: " << baseName << "_ift_watershed.png" << std::endl;

    // --- RECORTANDO UM SEGMENTO ESPECÍFICO COM IFT ---
    // Escolha um rótulo de segmento para recortar.
    // Você precisará inspecionar a imagem '_ift_watershed.png' para encontrar um rótulo interessante.
    // Para 'coffe-table.png', o rótulo do pixel (155, 350) pode ser parte da xícara de trás.
    int target_segment_label_ift = labels_ift[img.index(155, 350)]; // Exemplo: rótulo do pixel central da xícara de trás
    Pixel background_color_ift = {0, 0, 0}; // Cor de fundo preta para áreas fora do segmento recortado
    Image cropped_ift_segment = segmenter.cropSegment(labels_ift, target_segment_label_ift, background_color_ift);
    if (cropped_ift_segment.width > 0 && cropped_ift_segment.height > 0) {
        cropped_ift_segment.save("outputs/" + baseName + "_ift_watershed_cropped_segment.png");
        std::cout << "Segmento IFT-Watershed recortado (" << target_segment_label_ift << ") gerado: " 
                  << baseName << "_ift_watershed_cropped_segment.png" << std::endl;
    } else {
        std::cout << "Falha ao recortar segmento IFT-Watershed ou segmento não encontrado." << std::endl;
    }

    // --- CÓDIGO DE DEPURACÃO PARA O IFT-WATERSHED ---
    std::set<int> unique_ift_labels;
    for (int label : labels_ift) {
        unique_ift_labels.insert(label);
    }
    std::cout << "\n--- DEPURACAO IFT-WATERSHED ---" << std::endl;
    std::cout << "Número de sementes fornecidas: " << seeds.size() << std::endl;
    std::cout << "Número de rótulos únicos na saída: " << unique_ift_labels.size() << std::endl;
    if (unique_ift_labels.size() == 1) {
        std::cout << "ATENÇÃO: A segmentação resultou em uma única região." << std::endl;
    } else if (unique_ift_labels.size() < seeds.size()) {
        std::cout << "AVISO: O número de rótulos únicos (" << unique_ift_labels.size()
                  << ") é menor do que o número de sementes fornecidas (" << seeds.size() << ")." << std::endl;
        std::cout << "Isso é esperado no IFT-Watershed se sementes próximas competirem e uma delas prevalecer." << std::endl;
    }
    std::cout << "--- FIM DEPURACAO IFT-WATERSHED ---\n" << std::endl;

    // --- EXIBE OS TEMPOS DE EXECUÇÃO ---
    std::cout << "\n----------------------------------------" << std::endl;
    std::cout << "--- TEMPO DE EXECUÇÃO ---" << std::endl;
    std::cout << "Algoritmo Felzenszwalb: " << duration_fz.count() << " ms" << std::endl;
    std::cout << "Algoritmo IFT (com gradiente): " << duration_ift.count() << " ms" << std::endl;
    std::cout << "----------------------------------------\n" << std::endl;

    std::cout << "Processo de segmentação e recorte concluído!\n";
    return 0; // Retorna 0 indicando sucesso.
}