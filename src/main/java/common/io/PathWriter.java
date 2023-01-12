package common.io;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonIOException;
import com.google.gson.JsonObject;

import common.control.Path;
import common.control.PathSegment;
import common.io.json.InterpolatingDoubleJsonHandler;
import common.io.json.PathSegmentJsonHandler;
import common.io.json.Rotation2JsonHandler;
import common.io.json.SimpleMatrixJsonHandler;
import common.math.Rotation2;
import common.util.InterpolatingDouble;

import java.io.Flushable;
import java.io.IOException;
import java.io.Writer;
import org.ejml.simple.SimpleMatrix;

public final class PathWriter implements AutoCloseable, Flushable {
  private final Gson gson;
  private final Writer out;

  public PathWriter(Writer out) {
    this.gson =
        new GsonBuilder()
            .registerTypeAdapter(InterpolatingDouble.class, new InterpolatingDoubleJsonHandler())
            .registerTypeHierarchyAdapter(PathSegment.class, new PathSegmentJsonHandler())
            .registerTypeAdapter(Rotation2.class, new Rotation2JsonHandler())
            .registerTypeAdapter(SimpleMatrix.class, new SimpleMatrixJsonHandler())
            .enableComplexMapKeySerialization()
            .create();
    this.out = out;
  }

  public void write(Path path) throws IOException {
    try {
      JsonObject root = new JsonObject();
      root.add("segments", gson.toJsonTree(path.getSegments()));
      root.add("rotations", gson.toJsonTree(path.getRotationMap()));
      gson.toJson(root, out);
    } catch (JsonIOException e) {
      throw new IOException(e);
    }
  }

  @Override
  public void flush() throws IOException {
    out.flush();
  }

  @Override
  public void close() throws IOException {
    out.close();
  }
}
