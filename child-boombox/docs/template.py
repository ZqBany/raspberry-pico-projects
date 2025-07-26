# -*- coding: utf-8 -*-
import subprocess
import os
import urllib
import shutil
import csv
from urlparse import urlparse, parse_qs
import unicodedata
import re
import datetime
import time
import json
from multiprocessing.dummy import Pool as ThreadPool
from gimpfu import *

# import os
# os.chdir("SCRIPT_DIRECTORY")
# execfile("./template.py")

template_filename = "Card_stickers_A4_300dpi_template.xcf"
thumbnails_x_columns = [1, 614, 1231, 1843]
thumbnails_y_rows = [434, 1561, 2664]
desired_thumbnail_size_width = 542
desired_thumbnail_size_height = 520
desired_logo_size_height = 75
thumbnail_margin_to_not_overlap_width = 70
thumbnail_max_width = desired_thumbnail_size_width + 2*thumbnail_margin_to_not_overlap_width
logo_x_columns = []
logo_y_rows = [18, 1145, 2248]

for i in range(4):
    logo_x_columns.append(thumbnails_x_columns[i] + desired_thumbnail_size_width/2)

row_letters = ['A', 'B', 'C']

filename_base = os.getcwd()

now = datetime.datetime.now()
timestamp_str = now.strftime("%Y-%m-%d_%H_%M")

ffmpeg_base = "{0}\\audio_cards\\ffmpeg-2025-07-21-git-8cdb47e47a-essentials_build\\bin".format(filename_base)
ffprobe_path = os.path.join(ffmpeg_base, "ffprobe.exe")

audio_files_base = "{0}\\audio_cards\\audio_files".format(filename_base)
existing_files_by_yt_id = {}

def run_ffprobe(filepath):
    try:
        output = subprocess.check_output([
            ffprobe_path,
            '-v', 'quiet',
            '-show_format',
            '-print_format', 'json',
            filepath
        ])
        data = json.loads(output)
        comment = ""
        format_info = data.get("format", {})
        tags = format_info.get("tags", {})
        comment = tags.get("comment", "")

        if comment:  # Non-empty
            existing_files_by_yt_id[comment] = filename
    except subprocess.CalledProcessError:
        print("Error processing audio file:", filepath)

def populate_existing_files_by_yt_id():
    filepaths_list = []
    for filename in os.listdir(audio_files_base):
        filepaths_list.append("{0}\\{1}".format(audio_files_base, filename))
    pool = ThreadPool(10)  # max 10 concurrent threads
    results = pool.map(run_ffprobe, filepaths_list)
    pool.close()
    pool.join()

def replace_diacritics(s):
    s = unicodedata.normalize('NFKD', s)
    s = u"".join(c for c in s if not unicodedata.combining(c))
    replacements = {
        u"ł": u"l",
        u"Ł": u"L",
    }
    s = u"".join(replacements.get(c, c) for c in s)
    return s

def normalize_string(s):
    s = s.lower()
    s = re.sub(r'\s+', '_', s)
    s = replace_diacritics(s)
    return s

def scale(img, desired_height):
    img_width = pdb.gimp_image_width(img)
    img_height = pdb.gimp_image_height(img)
    img_ratio = float(desired_height) / img_height
    scaled_width = int(img_ratio * img_width)
    pdb.gimp_image_scale(img, scaled_width, desired_height)
    return scaled_width

def crop(thumbnail_img):
    thumbnail_width = pdb.gimp_image_width(thumbnail_img)
    thumbnail_height = pdb.gimp_image_height(thumbnail_img)
    if (thumbnail_width > thumbnail_max_width):
        crop_x_offset = (thumbnail_width - thumbnail_max_width) / 2
        pdb.gimp_image_crop(thumbnail_img, thumbnail_max_width, thumbnail_height, crop_x_offset, 0)
        scaled_width = thumbnail_max_width
    return scaled_width

def load_logo_png_as_layer(target_image, png_path):
    pdb.gimp_selection_none(target_image)  # ensure no selection active
    loaded_image = pdb.file_png_load(png_path, png_path)
    scale(loaded_image, desired_logo_size_height)
    loaded_layer = loaded_image.active_layer

    new_layer = pdb.gimp_layer_new(
        target_image,
        loaded_layer.width,
        loaded_layer.height,
        loaded_layer.type,
        "Logo layer",
        100,  # opacity 100%
        NORMAL_MODE
    )
    pdb.gimp_layer_add_alpha(new_layer)
    song_thumbnails_group = pdb.gimp_image_get_layer_by_name(target_image, "Logos")
    pdb.gimp_image_insert_layer(target_image, new_layer, song_thumbnails_group, 0)  # 0 - offset in group

    # Copy pixels from loaded image's layer
    pdb.gimp_edit_copy(loaded_layer)
    floating_sel = pdb.gimp_edit_paste(new_layer, True)  # True = paste into position
    pdb.gimp_floating_sel_anchor(floating_sel)

    # Explicitly set layer offset to (0, 0) so layer starts at top-left corner
    pdb.gimp_layer_set_offsets(new_layer, 0, 0)

    # Clean up by deleting the loaded image (to free memory)
    pdb.gimp_image_delete(loaded_image)
    return new_layer

def load_thumbnail_jpg_as_layer(target_image, jpg_path):
    pdb.gimp_selection_none(target_image) # ensure no selection active
    loaded_image = pdb.file_jpeg_load(jpg_path, jpg_path)
    scale(loaded_image, desired_thumbnail_size_height)
    result_thumbnail_width = crop(loaded_image)
    loaded_layer = loaded_image.active_layer

    new_layer = pdb.gimp_layer_new(
        target_image,
        loaded_layer.width,
        loaded_layer.height,
        loaded_layer.type,
        "Thumbnail layer",
        100,  # opacity 100%
        NORMAL_MODE
    )
    pdb.gimp_layer_add_alpha(new_layer)
    song_thumbnails_group = pdb.gimp_image_get_layer_by_name(target_image, "Thumbnails")
    pdb.gimp_image_insert_layer(target_image, new_layer, song_thumbnails_group, 0) # 0 - offset in group

    # Copy pixels from loaded image's layer
    pdb.gimp_edit_copy(loaded_layer)
    floating_sel = pdb.gimp_edit_paste(new_layer, True)  # True = paste into position
    pdb.gimp_floating_sel_anchor(floating_sel)

    # Explicitly set layer offset to (0, 0) so layer starts at top-left corner
    pdb.gimp_layer_set_offsets(new_layer, 0, 0)

    # Clean up by deleting the loaded image (to free memory)
    pdb.gimp_image_delete(loaded_image)
    return new_layer

def song_x_idx(song_idx):
    return song_idx % 4

def song_y_idx(song_idx):
    return song_idx / 4

def change_song_name(target_img, song_idx, song_title):
    song_name_layer_name = "{0}{1}".format(row_letters[song_y_idx(song_idx)], song_x_idx(song_idx))
    song_name_layer = pdb.gimp_image_get_layer_by_name(image, song_name_layer_name)
    pdb.gimp_text_layer_set_text(song_name_layer, song_title.encode('utf-8'))

def set_song_logo(target_img, song_idx, logo_filename):
    logo_layer = load_logo_png_as_layer(image, "{0}\\audio_cards\\logo\\{1}".format(filename_base, logo_filename))
    logo_x_offset = logo_x_columns[song_x_idx(song_idx)] - (logo_layer.width / 2)
    pdb.gimp_layer_set_offsets(logo_layer, logo_x_offset, logo_y_rows[song_y_idx(song_idx)])
    pdb.gimp_layer_set_name(logo_layer, "{0}{1} logo".format(row_letters[song_y_idx(song_idx)], song_x_idx(song_idx)))

def filled_template_name():
    return "songs_{0}.xcf".format(timestamp_str)

def filled_template_path():
    return "{0}\\audio_cards\\templates\\{1}".format(filename_base, filled_template_name())

def thumbnail_path(thumbnail_filename):
    return "{0}\\audio_cards\\thumbnail\\{1}".format(filename_base, thumbnail_filename)

def set_song_thumbnail(target_img, song_idx, thumbnail_filename):
    thumbnail_layer = load_thumbnail_jpg_as_layer(image, thumbnail_path(thumbnail_filename))
    thumbnail_x_offset = thumbnails_x_columns[song_x_idx(song_idx)] - ((thumbnail_layer.width - desired_thumbnail_size_width) / 2)
    pdb.gimp_layer_set_offsets(thumbnail_layer, thumbnail_x_offset, thumbnails_y_rows[song_y_idx(song_idx)])
    pdb.gimp_layer_set_name(thumbnail_layer, "{0}{1} thumbnail".format(row_letters[song_y_idx(song_idx)], song_x_idx(song_idx)))

def extract_youtube_id(url):
    if url.startswith(('youtu', 'www')):
        url = 'http://' + url

    parsed_url = urlparse(url)

    if 'youtube' in parsed_url.hostname:
        if parsed_url.path == '/watch':
            return parse_qs(parsed_url.query).get('v', [None])[0]
        elif parsed_url.path.startswith(('/embed/', '/v/')):
            return parsed_url.path.split('/')[2]

    elif 'youtu.be' in parsed_url.hostname:
        return parsed_url.path[1:]
    else:
        raise ValueError('Invalid YouTube URL')

def process_song(image, song_idx, author, title, youtube_id):
    print("Processing {0}: {1} - {2} - {3}".format(song_idx, replace_diacritics(author), replace_diacritics(title), youtube_id))
    thumbnail_filename = "{0}.jpg".format(youtube_id)
    if not os.path.isfile(thumbnail_path(thumbnail_filename)):
        urllib.urlretrieve("http://i3.ytimg.com/vi/{0}/maxresdefault.jpg".format(youtube_id), thumbnail_path(thumbnail_filename))
    try:
        existing_filename = existing_files_by_yt_id.get(youtube_id)
        if existing_filename:
            result = "Skip download - file already exists [{0}]".format(existing_filename)
        else:
            result = subprocess.check_output(['./audio_cards/yt-dlp.exe',
                                     '--extract-audio',
                                     '--audio-format', 'wav',
                                     '--postprocessor-args', "ffmpeg:-ar 44100 -ac 2 -c:a pcm_s16le -metadata IART='{0}' -metadata INAM='{1}' -metadata ICMT='{2}'".format(replace_diacritics(author), replace_diacritics(title), youtube_id),
                                     '--output', "./audio_cards/audio_files/{0} - {1}.%(ext)s".format(replace_diacritics(author), replace_diacritics(title)),
                                     '--ffmpeg-location', ffmpeg_base,
                                     "https://www.youtube.com/watch?v={0}".format(youtube_id)])
    except subprocess.CalledProcessError as exc:
        result = exc.output

    print(result)
    change_song_name(image, song_idx, title)
    set_song_thumbnail(image, song_idx, thumbnail_filename)
    set_song_logo(image, song_idx, "{0}.png".format(normalize_string(author)))

populate_existing_files_by_yt_id()
# for comment, filename in existing_files_by_yt_id.items():
#     print("Comment: {0} -> File: {1}".format(comment, filename))

shutil.copyfile(template_filename, filled_template_path())
image = pdb.gimp_xcf_load(0, filled_template_path(), filled_template_name())

with open('songs_to_add.csv', 'rb') as file_obj:
    heading = next(file_obj) # Skips the heading
    reader_obj = csv.reader(file_obj)
    song_idx = 0
    for row in reader_obj:
        row = [cell.decode('utf-8') for cell in row]
        if song_idx > 0:
            time.sleep(30)
        if song_idx >= 12:
            break
        author = row[0]
        title = row[1]
        youtube_id = extract_youtube_id(row[2])
        process_song(image, song_idx, author, title, youtube_id)
        song_idx = song_idx + 1

pdb.gimp_xcf_save(0, image, image.active_layer, filled_template_path(), filled_template_name())

