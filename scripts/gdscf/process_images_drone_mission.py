#!/usr/bin/python

# To parse arguments
import sys
# PostgreSQL driver
import psycopg2
# Image processing
from PIL import Image
# To plot histograms
import matplotlib.pyplot as plt
# To get k-means data
from sklearn.cluster import KMeans

def print_debug(message):
    if debug:
        print(message)

def connect():
    try:
        print_debug('### Connecting to the PostgreSQL database')
        global conn        
        
        # Get connection (tune connection string as needed).
        # TODO: add success/error checks
        conn = psycopg2.connect(host="192.168.1.3", dbname="gdscf", user="postgres", password="postgres")
    
        if debug:
            # Create a cursor.
            cur = conn.cursor()
            # Execute a statement.
            cur.execute('SELECT version()')
            # Display the PostgreSQL database server version.
            db_version = cur.fetchone()
            print_debug('### PostgreSQL database version: '+str(db_version))

    except (Exception, psycopg2.DatabaseError) as error:
        print_debug(error)
        raise Exception('### Connection to database not possible.')

# For table definitions check database_schema.sql
def get_images_for_drone_from_database(drone_id=1, mission_id=1):
    global conn
    global drone_images

    # Create a cursor.
    cur = conn.cursor()
    pg_select_query="SELECT secs, nsecs, image FROM drone_camera_blob WHERE drone_id = %s AND mission_id = %s" # AND secs > 4540 AND secs < 4543"
    cur.execute(pg_select_query,(str(drone_id),str(mission_id)))
    drone_images = cur.fetchall()
    print_debug("Retrieved "+str(len(drone_images))+" rows from database.")
    conn.close()
    print_debug('### Database connection closed.')

# Returns the center and weighted center from an array, after executing K-means with k=2
def get_k_means_centers(values_array=[]):
    # Compute K-means centers for the array
    kmeans_estimator = KMeans(n_clusters=2).fit([(x,0) for x in values_array])
    
    # Calculate regular center
    print_debug("Centers: "+str(kmeans_estimator.cluster_centers_))
    center = int(abs((kmeans_estimator.cluster_centers_[1,0]-kmeans_estimator.cluster_centers_[0,0])/2))
    print_debug("Center: "+str(center))
    
    # Calculate population per cluster
    kmeans_estimator_group0_pop = 0
    kmeans_estimator_group1_pop = 0
    for i in range(len(kmeans_estimator.labels_)):
        if kmeans_estimator.labels_[i] == 0:
            kmeans_estimator_group0_pop += 1
        else:
            kmeans_estimator_group1_pop += 1
    print_debug("Labels group count -> group0: "+str(kmeans_estimator_group0_pop)+" group1: "+str(kmeans_estimator_group1_pop))
    
    # Calculate weighted center
    if kmeans_estimator.cluster_centers_[0,0] < kmeans_estimator.cluster_centers_[1,0]:
        center_min = kmeans_estimator.cluster_centers_[0,0]
        center_min_pop = kmeans_estimator_group0_pop
        center_max = kmeans_estimator.cluster_centers_[1,0]
        center_max_pop = kmeans_estimator_group1_pop
    else:
        center_min = kmeans_estimator.cluster_centers_[1,0]
        center_min_pop = kmeans_estimator_group1_pop
        center_max = kmeans_estimator.cluster_centers_[0,0]
        center_max_pop = kmeans_estimator_group0_pop
    
    if center_min_pop > center_max_pop:
        # center = A + (B - A) * (Pop(B) / (Pop(A)+Pop(B)) )
        weighted_center = int(center_min+(center_max-center_min)*(center_max_pop/(center_min_pop+center_max_pop)))
    else:
        # center = B - (B - A) * (Pop(A) / (Pop(A)+Pop(B)) )
        weighted_center = int(center_max-(center_max-center_min)*(center_min_pop/(center_min_pop+center_max_pop)))
    print_debug("Weighted center: "+str(weighted_center))
    
    return center, weighted_center

def process_images_drone_mission(drone_id=1, mission_id=1, image_base_path="~", generate_images=False):
    width = 640
    height = 480
    row_length = 1920

    X256 = [i for i in range(256)]
    X512 = [i for i in range(512)]
    
    g_value_histogram_list = []
    exg_value_histogram_list =[]
    
    g_value_centers = []
    g_value_weighted_centers = []
    exg_value_centers = []
    exg_value_weighted_centers = []
        
    for drone_img_data in drone_images:
        print("\n### Processing second: "+str(drone_img_data[0]))

        g_value_histogram = [0 for i in range(256)]
        # Excess green values range from -512 to 512, but we will take negative values as zero
        exg_value_histogram = [0 for i in range(512)]
        
        min_g=256
        max_g=-256

        g_value_kmeans_array = []
        exg_value_kmeans_array = []
        
        for i in range(0, height*row_length, 3):
            r_value=int.from_bytes(drone_img_data[2][i],"little")
            g_value=int.from_bytes(drone_img_data[2][i+1],"little")
            b_value=int.from_bytes(drone_img_data[2][i+2],"little")
            #if debug:
            #    print("R: "+str(r_value)+" G: "+str(g_value)+" B: "+str(b_value))

            if g_value < min_g:
                min_g = g_value
            if g_value > max_g:
                max_g = g_value

            # Compute normalized ExG index value (negative values are zeroed out)
            exg_bw_normalized = int((2*g_value)-r_value-b_value)
            if (exg_bw_normalized < 0):
                #print_debug("ExG was less than zero: "+str(exg_bw_normalized)+" RGB: "+str(r_value)+" "+str(g_value)+" "+str(b_value))
                exg_bw_normalized = 0

            # Increment histogram for G component
            g_value_histogram[g_value] += 1
            # Add it to the array for K-means computation
            g_value_kmeans_array.append(g_value)
            
            # Increment histogram for ExG component
            exg_value_histogram[exg_bw_normalized] += 1
            # Add it to the array for K-means computation
            exg_value_kmeans_array.append(exg_bw_normalized)
        
        #print_debug("Max G: "+str(max_g)+" Min G: "+str(min_g))
        #print_debug("g_value_histogram: ")
        #print_debug(g_value_histogram)
        #print_debug("exg_value_histogram: ")
        #print_debug(exg_value_histogram)
        
        g_value_histogram_list.append(g_value_histogram)
        exg_value_histogram_list.append(exg_value_histogram)

        # Compute K-means centers for the RGB image
        print_debug("## Computing K-means for the RGB image")
        g_value_center, g_value_weighted_center = get_k_means_centers(g_value_kmeans_array)
        g_value_centers.append(g_value_center)
        g_value_weighted_centers.append(g_value_weighted_center)

        # Compute K-means centers for the ExG image
        print_debug("## Computing K-means for the ExG image")
        exg_value_center, exg_value_weighted_center = get_k_means_centers(exg_value_kmeans_array)
        exg_value_centers.append(exg_value_center)
        exg_value_weighted_centers.append(exg_value_weighted_center)
        
        if generate_images == True:
            drone_img = Image.new('RGB', (width, height))
            drone_img_grayscale = Image.new('RGB', (width, height))
            drone_img_bw_rgb = Image.new('1', (width, height))
            drone_img_bw_exg = Image.new('1', (width, height))
            drone_img_bw_weighted_rgb = Image.new('1', (width, height))
            drone_img_bw_weighted_exg = Image.new('1', (width, height))
            drone_img_2x3 = Image.new('RGB', (3*width, 2*height))

            # RGB and green scale images
            rgb_tuples = []
            gs_tuples  = []
            # Monochrome images with filtered out pixels according to calculated centroid values
            bw_rgb_tuples = []
            bw_exg_tuples = []
            bw_weighted_rgb_tuples = []
            bw_weighted_exg_tuples = []

            for i in range(0, height*row_length, 3):
                r_value=int.from_bytes(drone_img_data[2][i],"little")
                g_value=int.from_bytes(drone_img_data[2][i+1],"little")
                b_value=int.from_bytes(drone_img_data[2][i+2],"little")
                # Compute normalized ExG index value
                exg_bw_normalized = int((2*g_value)-r_value-b_value)
                if (exg_bw_normalized < 0):
                    #print_debug("ExG was less than zero: "+str(exg_bw_normalized)+" RGB: "+str(r_value)+" "+str(g_value)+" "+str(b_value))
                    exg_bw_normalized = 0
                
                rgb_tuples.append((r_value,g_value,b_value))
                # We force R=0 and B=0 to get a green scale image with ExG values
                gs_tuples.append((0,exg_bw_normalized,0))

                if (g_value >= g_value_center):
                    bw_rgb_tuples.append(1)
                else:
                    bw_rgb_tuples.append(0)

                if (g_value >= g_value_weighted_center):
                    bw_weighted_rgb_tuples.append(1)
                else:
                    bw_weighted_rgb_tuples.append(0)

                if (exg_bw_normalized >= exg_value_center):
                    bw_exg_tuples.append(1)
                else:
                    bw_exg_tuples.append(0)

                if (exg_bw_normalized >= exg_value_weighted_center):
                    bw_weighted_exg_tuples.append(1)
                else:
                    bw_weighted_exg_tuples.append(0)

            # Original RGB image
            drone_img.putdata(rgb_tuples)
            drone_img_2x3.paste(drone_img,(0,0))
            # BW (gray scale) from ExG index
            drone_img_grayscale.putdata(gs_tuples)
            drone_img_2x3.paste(drone_img_grayscale,(0,height))
            # BW (monochrome) filtered from RGB with k-means centroid
            drone_img_bw_rgb.putdata(bw_rgb_tuples)
            drone_img_2x3.paste(drone_img_bw_rgb,(width,0))
            # BW (monochrome) filtered from RGB with k-means centroid with weights by cluster population
            drone_img_bw_weighted_rgb.putdata(bw_weighted_rgb_tuples)
            drone_img_2x3.paste(drone_img_bw_weighted_rgb,(2*width,0))
            # BW (monochrome) filtered from ExG with k-means centroid
            drone_img_bw_exg.putdata(bw_exg_tuples)
            drone_img_2x3.paste(drone_img_bw_exg,(width,height))
            # BW (monochrome) filtered from ExG with k-means centroid with weights by cluster population
            drone_img_bw_weighted_exg.putdata(bw_weighted_exg_tuples)
            drone_img_2x3.paste(drone_img_bw_weighted_exg,(2*width,height))
            # Save full image to disk
            drone_img_2x3_path=image_base_path+"/vegetation_indexes__drone_"+str(drone_id)+"_mission_"+str(mission_id)+"_image_"+str(drone_img_data[0])+"."+str(drone_img_data[1])+".png"
            drone_img_2x3.save(drone_img_2x3_path)
            
    print("\n### Image processing finished.")
    
    # Plot histograms
    for idx in range(len(g_value_histogram_list)):
        plt.subplot(1,2,1)
        plt.plot(X256, g_value_histogram_list[idx], fillstyle="none")
        plt.title("Green values histogram.")
    
        plt.subplot(1,2,2)
        plt.plot(X512, exg_value_histogram_list[idx], fillstyle="none")
        plt.title("Excess green values histogram.")

        plt.savefig(image_base_path+"/histogram_plots__index_"+str(idx))
        plt.clf()

    g_avg_value_cutoff = int(sum(g_value_centers) / len(g_value_centers))
    exg_avg_value_cutoff = int(sum(exg_value_centers) / len(exg_value_centers))

    g_weighted_avg_value_cutoff = int(sum(g_value_weighted_centers) / len(g_value_weighted_centers))
    exg_weighted_avg_value_cutoff = int(sum(exg_value_weighted_centers) / len(exg_value_weighted_centers))

    print("# Images processed: "+str(len(drone_images)))
    print("# Centers for RGB: "+str(g_value_centers)  +"\n(weighted): "+str(g_value_weighted_centers))
    print("# Centers for ExG: "+str(exg_value_centers)+"\n(weighted): "+str(exg_value_weighted_centers))
    print("# RGB value average cutoff: "+str(g_avg_value_cutoff)  +"  (weighted): "+str(g_weighted_avg_value_cutoff))
    print("# ExG average value cutoff: "+str(exg_avg_value_cutoff)+"  (weighted): "+str(exg_weighted_avg_value_cutoff))


if __name__ == '__main__':
    debug = True
    generate_images = False
    print_debug("### Arguments passed: "+str(len(sys.argv)-1))

    # Argument order:
    #  1- drone_id
    #  2- mission_id
    #  3- image_base_path
    #  4- generate_images

    if len(sys.argv) > 4:
        if int(sys.argv[4]) == 1:
            generate_images = True

    if len(sys.argv) > 3:
        # We assume first argument is drone_id
        drone_id = int(sys.argv[1])
        # We assume second argument is mission_id
        mission_id = int(sys.argv[2])
        # We assume third argument is image_base_path
        image_base_path = str(sys.argv[3])
    else:
        print('ERROR: Not enough arguments used.\nUsage: image_creator.py drone_id mission_id image_base_path')
        quit()
    
    print_debug("Generate images? "+str(generate_images))
    connect()

    get_images_for_drone_from_database(drone_id, mission_id)

    process_images_drone_mission(drone_id, mission_id, image_base_path, generate_images)

    if conn is not None:
        conn.close()
        print_debug('### Database connection closed.')
    else:
        print_debug('### Database connection was null.')

