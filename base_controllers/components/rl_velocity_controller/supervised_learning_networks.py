import torch
from torch.utils.data import Dataset
import random
import os

class CustomDataset(Dataset):
    def __init__(self, max_size=None):
        self.data = []
        self.labels = []
        self.max_size = max_size

    def add_sample(self, input_data, label):
        # There is a problem! we append (num_envs, features) as one element inside the list, not N!

        # Save only random 128 element from the input_data and label
        random_idx = random.sample(range(input_data.size(0)), min(128, input_data.size(0)))
        input_data_cpu = input_data[random_idx].clone().detach().cpu()
        label_cpu = label[random_idx].clone().detach().cpu()

        #input_data_cpu = input_data.clone().detach().cpu()
        #label_cpu = label.clone().detach().cpu()
        self.data.append(input_data_cpu)
        self.labels.append(label_cpu)

        # Check if the buffer exceeds the maximum size
        if self.max_size is not None and len(self.data) > self.max_size:
            # Remove a random sample to maintain the buffer size
            idx_to_remove = random.randint(0, len(self.data) - 1)
            del self.data[idx_to_remove]
            del self.labels[idx_to_remove]


    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        return self.data[idx], self.labels[idx]

"""
class CustomDataset(Dataset):
    def __init__(self, max_size=None):
        self.data = None  # Will store as a single tensor
        self.labels = None
        self.max_size = max_size
        self.current_size = 0

    def add_sample(self, input_data, label):
        input_cpu = input_data.detach().cpu()
        label_cpu = label.detach().cpu()
        
        if self.data is None:
            # Initialize tensors
            self.data = input_cpu
            self.labels = label_cpu
        else:
            # Concatenate new data
            self.data = torch.cat([self.data, input_cpu], dim=0)
            self.labels = torch.cat([self.labels, label_cpu], dim=0)
        
        self.current_size = self.data.size(0)
        
        # Handle max_size constraint
        if self.max_size is not None and self.current_size > self.max_size:
            # Keep random subset
            indices = torch.randperm(self.current_size)[:self.max_size]
            self.data = self.data[indices]
            self.labels = self.labels[indices]
            self.current_size = self.max_size

    def __len__(self):
        return self.current_size if self.data is not None else 0

    def __getitem__(self, idx):
        return self.data[idx], self.labels[idx]
"""


class SimpleNN(torch.nn.Module):
    def __init__(self, in_features, out_features):
        super().__init__()
        self.fc1 = torch.nn.Linear(in_features, 128)
        self.fc2 = torch.nn.Linear(128, 64)
        self.fc3 = torch.nn.Linear(64, out_features)

        self.dataset = CustomDataset(max_size=20000)


    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        x = self.fc3(x)
        return x
    

    def train_network(self, batch_size=512, epochs=1000, learning_rate=1e-3, device='cpu'):
        with torch.inference_mode(False):
            with torch.enable_grad():
                # Define optimizer and loss function
                optimizer = torch.optim.Adam(self.parameters(), lr=learning_rate)
                loss_fn = torch.nn.MSELoss()
    
                # Create a DataLoader for batching
                dataloader = torch.utils.data.DataLoader(
                    self.dataset,
                    batch_size=batch_size,
                    shuffle=True
                )
    
                # Training loop
                self.train()
                for epoch in range(epochs):
                    for inputs, targets in dataloader:
                    
                        # Forward pass
                        inputs = inputs.view(-1, inputs.size(-1)).to(device)
                        targets = targets.view(-1, targets.size(-1)).to(device)
                        predictions = self(inputs)
    
                        loss = loss_fn(predictions, targets)
    
                        # Backward pass and optimization
                        optimizer.zero_grad()
                        loss.backward()
                        optimizer.step()
    
                    print(f"Epoch {epoch + 1}/{epochs}, Loss: {loss.item()}")
        self.eval()
        print("Training complete. Model set to evaluation mode.")


    def save_network(self, filepath, device='cpu'):
        """Save the network state dict to a file."""
        # Create directory if it doesn't exist
        os.makedirs(os.path.dirname(filepath) if os.path.dirname(filepath) else '.', exist_ok=True)
        
        # Move model to CPU for saving (optional, saves GPU memory)
        original_device = next(self.parameters()).device
        #self.cpu()
        
        # Save the state dict
        torch.save({
            'model_state_dict': self.state_dict(),
            'input_features': self.fc1.in_features,
            'output_features': self.fc3.out_features,
        }, filepath)
        
        print(f"Network saved to {filepath}")
        
        # Move model back to original device
        #self.to(original_device)


def load_network(filepath, device='cpu'):
    """Load the network state dict from a file."""
    if not os.path.isfile(filepath):
        raise FileNotFoundError(f"No such file: '{filepath}'")
    
    checkpoint = torch.load(filepath, map_location=device)
    
    input_features = checkpoint.get('input_features')
    output_features = checkpoint.get('output_features')
    if input_features is None or output_features is None:
        raise ValueError("Checkpoint does not contain model architecture information.")
    # Reinitialize the model with the correct architecture
    model = SimpleNN(input_features, output_features)
    model.load_state_dict(checkpoint['model_state_dict'])
    model.to(device)
    model.eval()  # Set the model to evaluation mode
    
    print(f"Network loaded from {filepath}")
    return model
    
    


"""model = SimpleNN(10, 2)
x = torch.randn(4, 10)
y = torch.randn(4, 2)
out = model(x)
print("out.requires_grad:", out.requires_grad)
loss = torch.nn.MSELoss()(out, y)
print("loss.requires_grad:", loss.requires_grad)"""