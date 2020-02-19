from AdvancedLanedFinder import *
import yaml

def dict_to_numpy(d):
  rows = d['rows']
  cols = d['cols']
  data = np.array(d['data'])
  return data.reshape((rows, cols))

def load_params():
  ap = AdvancedLaneFinderParams()
  ip = ImagePipelineParams()
  with open('project2_config.yaml','r') as file:
    config = yaml.load(file, Loader=yaml.FullLoader)

    ip.video_filename = config['video_filename']
    ip.frame_rate_hz = config['frame_rate_Hz']
    ip.trackbar_window = config['trackbar_window']
    ip.image_window = config['image_window']
    ip.loop_video = config['loop_video']
    ip.record_video = config['record_video']
    ip.show_trackbars = config['show_trackbars']
    
    ap.trackbar_window = config['trackbar_window']
    ap.K = dict_to_numpy(config['K'])
    ap.D = dict_to_numpy(config['D'])
    ap.M = dict_to_numpy(config['M'])
    ap.M_inv = np.linalg.inv(ap.M)
    ap.lp = LaneLineParams()
    ap.lp.xm_per_pix = config['xm_per_pix']
    ap.lp.ym_per_pix = config['ym_per_pix']
    ap.lp.margin = config['margin']
    ap.lp.minpix = config['minpix']
    ap.lp.nwindows = config['nwindows']
    ap.min_sobel = config['min_sobel']
    ap.max_sobel = config['max_sobel']
    ap.min_s = config['min_s']
    ap.max_s = config['max_s']
    ap.min_l = config['min_l']
    ap.max_l = config['max_l']
    ap.show_trackbars = config['show_trackbars']
    ap.show_warped = config['show_warped']
    ap.initialized = True

  return ap, ip


def main():

  ap, ip  = load_params()

  finder = AdvancedLaneFinder(ap, ip)

  finder.Run()


if __name__ == "__main__":
    main()


