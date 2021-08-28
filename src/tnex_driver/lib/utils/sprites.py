import pygame


class Text(pygame.sprite.Sprite):
    def __init__(self, text, position, group, layer, color=(255, 255, 255), font=None):
        if font is not None:
            self.font = font
        else:
            self.font = pygame.font.Font(pygame.font.get_default_font(), 15)
        self.color = color

        self.image = self.font.render(text, False, self.color)
        self.rect = self.image.get_rect(topleft=position)
        self._layer = layer
        pygame.sprite.Sprite.__init__(self, group)

    def updateText(self, text):
        self.image = self.font.render(text, False, self.color)


class Image(pygame.sprite.Sprite):
    def __init__(self, surface, position, group, layer):
        self.image = surface
        self.rect = self.image.get_rect(topleft=position)
        self._layer = layer
        pygame.sprite.Sprite.__init__(self, group)

    def updateImage(self, surface):
        self.image = surface
