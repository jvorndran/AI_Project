import pygame
import sys
from distance_optimized import WarehouseSimulation

# Assume distance_optimized.py contains the WarehouseSimulation class

pygame.init()

screen_width, screen_height = 600, 300  # Increased screen size for better layout
screen = pygame.display.set_mode((screen_width, screen_height))
pygame.display.set_caption("Simulation Setup")

color_inactive = pygame.Color('lightskyblue3')
color_active = pygame.Color('dodgerblue2')
color_text = pygame.Color('black')
background_color = pygame.Color('white')
button_color = pygame.Color('lightgrey')
button_hover_color = pygame.Color('grey')

font = pygame.font.Font(None, 32)

input_boxes = {
    "grid_size": {"rect": pygame.Rect(250, 50, 140, 32), "text": "", "active": False, "label": "Grid Size:"},
    "num_robots": {"rect": pygame.Rect(250, 100, 140, 32), "text": "", "active": False, "label": "Number of Robots:"},
    "num_obstacles": {"rect": pygame.Rect(250, 150, 140, 32), "text": "", "active": False, "label": "Number of Obstacles:"}
}
button_rect = pygame.Rect(200, 220, 200, 50)
button_text = "Start Simulation"

def draw_setup_screen(mouse_pos):
    screen.fill(background_color)
    for key, box in input_boxes.items():
        label_surface = font.render(box["label"], True, color_text)
        screen.blit(label_surface, (box["rect"].x - 220, box["rect"].y + 5))
        txt_surface = font.render(box["text"], True, color_text)
        screen.blit(txt_surface, (box["rect"].x + 5, box["rect"].y + 5))
        pygame.draw.rect(screen, color_active if box["active"] else color_inactive, box["rect"], 2)

    # Draw the start button
    button_color_current = button_hover_color if button_rect.collidepoint(mouse_pos) else button_color
    pygame.draw.rect(screen, button_color_current, button_rect)
    button_txt_surface = font.render(button_text, True, color_text)
    screen.blit(button_txt_surface, (button_rect.x + 20, button_rect.y + 10))

    pygame.display.flip()

def is_integer(s):
    try:
        int(s)
        return True
    except ValueError:
        return False

clock = pygame.time.Clock()
done = False
mouse_pos = (0, 0)

while not done:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if button_rect.collidepoint(event.pos):
                if all(is_integer(input_boxes[key]["text"]) for key in input_boxes):
                    done = True  # Proceed only if all inputs are integers
            else:
                for key, box in input_boxes.items():
                    if box["rect"].collidepoint(event.pos):
                        box["active"] = True
                        current_input = key  # Set the current input box
                    else:
                        box["active"] = False
        elif event.type == pygame.KEYDOWN and current_input:
            if event.key == pygame.K_BACKSPACE:
                input_boxes[current_input]["text"] = input_boxes[current_input]["text"][:-1]
            elif event.unicode.isdigit():
                input_boxes[current_input]["text"] += event.unicode

    mouse_pos = pygame.mouse.get_pos()
    draw_setup_screen(mouse_pos)
    clock.tick(30)

# Ensure all inputs are integers
grid_size = int(input_boxes["grid_size"]["text"]) if is_integer(input_boxes["grid_size"]["text"]) else 50
num_robots = int(input_boxes["num_robots"]["text"]) if is_integer(input_boxes["num_robots"]["text"]) else 6
num_obstacles = int(input_boxes["num_obstacles"]["text"]) if is_integer(input_boxes["num_obstacles"]["text"]) else 30
cell_size = 20

# Start the simulation with provided parameters
print(f'Starting simulation with grid size {grid_size}, {num_robots} robots, and {num_obstacles} obstacles.')
sim = WarehouseSimulation(grid_size, cell_size, num_robots, num_obstacles)
sim.run()
