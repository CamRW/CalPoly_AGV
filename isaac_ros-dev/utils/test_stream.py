import torch
import numpy as np
import torch.nn as nn
import cv2
import albumentations
import time

class EncoderBlock(nn.Module):        
    # Consists of Conv -> ReLU -> MaxPool
    def __init__(self, in_chans, out_chans, layers=2, sampling_factor=2, padding="same"):
        super().__init__()
        self.encoder = nn.ModuleList()
        self.encoder.append(nn.Conv2d(in_chans, out_chans, 3, 1, padding=padding))
        self.encoder.append(nn.ReLU())
        for _ in range(layers-1):
            self.encoder.append(nn.Conv2d(out_chans, out_chans, 3, 1, padding=padding))
            self.encoder.append(nn.ReLU())
        self.mp = nn.MaxPool2d(sampling_factor)
    def forward(self, x):
        #print("Encoder forward", x.shape)
        for enc in self.encoder:
            x = enc(x)
        mp_out = self.mp(x)
        return mp_out, x

class DecoderBlock(nn.Module):
    # Consists of 2x2 transposed convolution -> Conv -> relu
    def __init__(self, in_chans, out_chans, layers=2, skip_connection=True, sampling_factor=2, padding="same"):
        super().__init__()
        skip_factor = 1 if skip_connection else 2
        self.decoder = nn.ModuleList()
        self.tconv = nn.ConvTranspose2d(in_chans, in_chans//2, sampling_factor, sampling_factor)

        self.decoder.append(nn.Conv2d(in_chans//skip_factor, out_chans, 3, 1, padding=padding))
        self.decoder.append(nn.ReLU())

        for _ in range(layers-1):
            self.decoder.append(nn.Conv2d(out_chans, out_chans, 3, 1, padding=padding))
            self.decoder.append(nn.ReLU())

        self.skip_connection = skip_connection
        self.padding = padding
    def forward(self, x, enc_features=None):
        x = self.tconv(x)
        if self.skip_connection:
            if self.padding != "same":
                # Crop the enc_features to the same size as input
                w = x.size(-1)
                c = (enc_features.size(-1) - w) // 2
                enc_features = enc_features[:,:,c:c+w,c:c+w]
            x = torch.cat((enc_features, x), dim=1)
        for dec in self.decoder:
            x = dec(x)
        return x

class UNet(nn.Module):
    def __init__(self, nclass=1, in_chans=1, depth=5, layers=2, sampling_factor=2, skip_connection=True, padding="same"):
        super().__init__()
        self.encoder = nn.ModuleList()
        self.decoder = nn.ModuleList()

        out_chans = 64
        for _ in range(depth):
            self.encoder.append(EncoderBlock(in_chans, out_chans, layers, sampling_factor, padding))
            in_chans, out_chans = out_chans, out_chans*2

        out_chans = in_chans // 2
        for _ in range(depth-1):
            self.decoder.append(DecoderBlock(in_chans, out_chans, layers, skip_connection, sampling_factor, padding))
            in_chans, out_chans = out_chans, out_chans//2
        # Add a 1x1 convolution to produce final classes
        self.logits = nn.Conv2d(in_chans, nclass, 1, 1)

    def forward(self, x):
        #print("Forward shape ", x.shape)
        encoded = []
        for enc in self.encoder:
            x, enc_output = enc(x)
            encoded.append(enc_output)
        x = encoded.pop()
        for dec in self.decoder:
            enc_output = encoded.pop()
            x = dec(x, enc_output)

        # Return the logits
        #print("Logits shape ", self.logits(x).shape)
        return self.logits(x)
    

transform = albumentations.Compose([
    albumentations.Resize(224, 224, always_apply=True),
    albumentations.Normalize(
            mean=[0.45734706, 0.43338275, 0.40058118],
            std=[0.23965294, 0.23532275, 0.2398498],
            always_apply=True)
])
    

if  __name__ == '__main__':
    WIDTH = 640
    HEIGHT = 480

    vid = cv2.VideoCapture(0, cv2.CAP_V4L2)
    vid.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    vid.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

    vid.set(cv2.CAP_PROP_BUFFERSIZE,1)
    font = cv2.FONT_HERSHEY_SIMPLEX

    # Green color in BGR
    color = (0, 255, 0)
    
    # Line thickness of 9 px
    thickness = 9

    dtype = torch.float32
    if torch.cuda.is_available():
        device = torch.device('cuda:0')
    else:
        device = torch.device('cpu')
    print('using device:', device)

    unet = UNet(in_chans=3, depth=3, layers=1, skip_connection=True)

    unet.load_state_dict(torch.load('unet.pkl'))

    sigmoid = nn.Sigmoid()

    # Make sample predictions
    unet.eval()
    unet.to(device)
    with torch.no_grad():
        while True:
            tic = time.perf_counter()
            ret, frame = vid.read()
            orig_frame = frame.copy()
            orig_frame = cv2.resize(orig_frame, (224,224), interpolation = cv2.INTER_AREA)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = transform(image=frame)['image']
            frame = np.transpose(frame, (2, 0, 1))
            frame = torch.tensor(frame, dtype=torch.float32)
            frame = frame.unsqueeze(0).to(device)
            infer = unet(frame).squeeze()
            # print(infer)
            infer = sigmoid(infer)
            #print(infer)

            infer = infer.detach().cpu().numpy()
            infer = np.where(infer >= .5, 0, 255).astype(np.uint8)

            #print(infer)
            
            #print(infer)

             # Generate intermediate image; use morphological closing to keep parts together
            inter = cv2.morphologyEx(infer, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))

            # Find largest contour in intermediate image
            cnts, _ = cv2.findContours(inter, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            cnt = max(cnts, key=cv2.contourArea)
            #print(cnt)

            # Output
            out = np.zeros(infer.shape, np.uint8)
            cv2.drawContours(out, [cnt], -1, 255, cv2.FILLED)
            out = cv2.bitwise_and(inter, out)

            out = cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)
            
            added_image = cv2.addWeighted(orig_frame,0.5,out,0.5,0)
            toc = time.perf_counter()
            print(f"Time taken to for full process: {toc - tic:0.4f} seconds")


            cv2.imshow('road_finder', added_image)
            if cv2.waitKey(50) == ord('q'):
                break


