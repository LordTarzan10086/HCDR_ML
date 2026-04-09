import csv
import matplotlib.pyplot as plt
import numpy as np
import os
import glob

def main():
    # Find latest output folder
    output_root = r"c:\Users\A\Desktop\code\C++\HCDR_v6-codex--branch_for_WFW\cpp_workspace_static_mode1\output"
    folders = sorted([os.path.join(output_root, d) for d in os.listdir(output_root) if os.path.isdir(os.path.join(output_root, d))])
    if not folders:
        print("No output folders found.")
        return
    latest_dir = folders[-1]
    csv_file = os.path.join(latest_dir, "mode1_platform_fixedpsi_45deg_gamma.csv")
    
    if not os.path.exists(csv_file):
        print("CSV file not found:", csv_file)
        return
        
    print("Reading", csv_file)
    
    x_list = []
    y_list = []
    gamma_list = []
    
    with open(csv_file, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            x_m = float(row['x_m'])
            y_m = float(row['y_m'])
            gamma_n = float(row['gamma_n']) if row['gamma_n'].strip() != 'nan' else -1000.0
            rank = int(row['rank'])
            # Treat rank deficient points as highly negative gamma so contour lines avoid them
            if rank < 3:
                gamma_n = -1000.0
                
            x_list.append(x_m)
            y_list.append(y_m)
            gamma_list.append(gamma_n)

    x_arr = np.array(x_list)
    y_arr = np.array(y_list)
    g_arr = np.array(gamma_list)
    
    # Get unique sorted coordinates
    unique_x = np.sort(np.unique(x_arr))
    unique_y = np.sort(np.unique(y_arr))
    
    X, Y = np.meshgrid(unique_x, unique_y)
    Z = np.full(X.shape, -1000.0)
    
    # Fill grid
    for i in range(len(x_arr)):
        idx_x = np.where(unique_x == x_arr[i])[0][0]
        idx_y = np.where(unique_y == y_arr[i])[0][0]
        Z[idx_y, idx_x] = g_arr[i]

    plt.figure(figsize=(10, 8))
    
    # We contour the boundary at gamma = 0
    # Specifically using a small epsilon to avoid discrete noise at the very edge of feasibility
    # Plot heatmap of feasible region only to match the C++ style
    Z_feasible = np.where(Z >= 0, Z, np.nan)
    cmap = plt.cm.jet
    mesh = plt.pcolormesh(X, Y, Z_feasible, cmap=cmap, shading='auto', vmin=0, vmax=250)
    cbar = plt.colorbar(mesh)
    cbar.set_label('Gamma [N]')

    # Add red contour line at gamma = 0
    # Use levels=[0.001] if data is very sparse at the bridge between valid/invalid
    CS = plt.contour(X, Y, Z, levels=[0.001], colors='red', linewidths=4.0)
    
    # Format plot
    plt.title('Mode 1 Wrench-Feasible Workspace (WFW)\nFixed Psi = 45 deg, Red Line = Gamma 0 Boundary')
    plt.xlabel('Platform X [m]')
    plt.ylabel('Platform Y [m]')
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.axis('equal')
    
    # Export
    out_png = os.path.join(latest_dir, "python_plot_wfw_boundary.png")
    plt.tight_layout()
    plt.savefig(out_png, dpi=300)
    print("Saved WFW contour to:", out_png)

if __name__ == "__main__":
    main()
